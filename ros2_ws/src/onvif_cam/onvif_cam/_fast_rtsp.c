#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stdint.h>
#include <stdio.h>

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/log.h>
#include <libswscale/swscale.h>

typedef struct {
    PyObject_HEAD
    AVFormatContext *fmt;
    AVCodecContext *dec;
    struct SwsContext *sws;
    AVPacket *pkt;
    AVFrame *frame;
    int video_idx;
    int sws_w;
    int sws_h;
    int sws_fmt;
    int got_keyframe;
    volatile int aborted;
} FastRtspObject;

static int interrupt_cb(void *opaque) {
    FastRtspObject *self = (FastRtspObject *)opaque;
    return self->aborted ? 1 : 0;
}

static void free_resources(FastRtspObject *self) {
    if (self->sws) {
        sws_freeContext(self->sws);
        self->sws = NULL;
    }
    if (self->frame) {
        av_frame_free(&self->frame);
    }
    if (self->pkt) {
        av_packet_free(&self->pkt);
    }
    if (self->dec) {
        avcodec_free_context(&self->dec);
    }
    if (self->fmt) {
        avformat_close_input(&self->fmt);
    }
    self->video_idx = -1;
    self->sws_w = 0;
    self->sws_h = 0;
    self->sws_fmt = -1;
    self->got_keyframe = 0;
}

static void FastRtsp_dealloc(FastRtspObject *self) {
    self->aborted = 1;
    free_resources(self);
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static int FastRtsp_init(FastRtspObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"url", "transport", "timeout_ms", NULL};
    const char *url = NULL;
    const char *transport = "tcp";
    int timeout_ms = 3000;

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|si", kwlist, &url, &transport, &timeout_ms)) {
        return -1;
    }

    free_resources(self);
    self->aborted = 0;
    self->video_idx = -1;
    self->sws_fmt = -1;

    int err = 0;
    const char *err_stage = NULL;

    Py_BEGIN_ALLOW_THREADS
    self->fmt = avformat_alloc_context();
    if (!self->fmt) {
        err = AVERROR(ENOMEM);
        err_stage = "avformat_alloc_context";
    } else {
        self->fmt->interrupt_callback.callback = interrupt_cb;
        self->fmt->interrupt_callback.opaque = self;

        AVDictionary *opts = NULL;
        if (transport && transport[0]) {
            av_dict_set(&opts, "rtsp_transport", transport, 0);
        }
        av_dict_set(&opts, "fflags", "nobuffer", 0);
        av_dict_set(&opts, "flags", "low_delay", 0);
        av_dict_set(&opts, "probesize", "32768", 0);
        av_dict_set(&opts, "analyzeduration", "0", 0);

        char timeout_buf[32];
        snprintf(timeout_buf, sizeof(timeout_buf), "%lld", (long long)timeout_ms * 1000LL);
#if LIBAVFORMAT_VERSION_MAJOR >= 59
        av_dict_set(&opts, "timeout", timeout_buf, 0);
#else
        av_dict_set(&opts, "stimeout", timeout_buf, 0);
#endif

        err = avformat_open_input(&self->fmt, url, NULL, &opts);
        av_dict_free(&opts);
        if (err < 0) {
            err_stage = "avformat_open_input";
        }
    }

    if (err >= 0) {
        self->fmt->flags |= AVFMT_FLAG_NOBUFFER | AVFMT_FLAG_FLUSH_PACKETS;
        err = avformat_find_stream_info(self->fmt, NULL);
        if (err < 0) {
            err_stage = "avformat_find_stream_info";
        }
    }

    if (err >= 0) {
        for (unsigned i = 0; i < self->fmt->nb_streams; i++) {
            if (self->fmt->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                self->video_idx = (int)i;
                break;
            }
        }
        if (self->video_idx < 0) {
            err = AVERROR_STREAM_NOT_FOUND;
            err_stage = "find_video_stream";
        }
    }

    if (err >= 0) {
        AVCodecParameters *par = self->fmt->streams[self->video_idx]->codecpar;
        const AVCodec *codec = avcodec_find_decoder(par->codec_id);
        if (!codec) {
            err = AVERROR_DECODER_NOT_FOUND;
            err_stage = "avcodec_find_decoder";
        } else {
            self->dec = avcodec_alloc_context3(codec);
            if (!self->dec) {
                err = AVERROR(ENOMEM);
                err_stage = "avcodec_alloc_context3";
            } else {
                avcodec_parameters_to_context(self->dec, par);
                /* Disable frame-level multithreading to eliminate N-frame reorder/pipeline lag. */
                self->dec->thread_count = 1;
                self->dec->thread_type = 0;
                self->dec->flags |= AV_CODEC_FLAG_LOW_DELAY;

                err = avcodec_open2(self->dec, codec, NULL);
                if (err < 0) {
                    err_stage = "avcodec_open2";
                }
            }
        }
    }

    if (err >= 0) {
        self->pkt = av_packet_alloc();
        self->frame = av_frame_alloc();
        if (!self->pkt || !self->frame) {
            err = AVERROR(ENOMEM);
            err_stage = "av_packet_or_frame_alloc";
        }
    }

    if (err < 0) {
        free_resources(self);
    }
    Py_END_ALLOW_THREADS

    if (err < 0) {
        char errbuf[128];
        av_strerror(err, errbuf, sizeof(errbuf));
        PyErr_Format(PyExc_RuntimeError, "%s failed: %s", err_stage ? err_stage : "rtsp", errbuf);
        return -1;
    }
    return 0;
}

static PyObject *FastRtsp_read(FastRtspObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"convert", NULL};
    int convert = 1;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|p", kwlist, &convert)) {
        return NULL;
    }

    if (!self->fmt || !self->dec || self->aborted) {
        return Py_BuildValue("(OOii)", Py_False, Py_None, 0, 0);
    }

    int got_frame = 0;
    int width = 0;
    int height = 0;
    int pix_fmt = -1;

    Py_BEGIN_ALLOW_THREADS
    while (!self->aborted) {
        int ret = av_read_frame(self->fmt, self->pkt);
        if (ret < 0) {
            break;
        }
        if (self->pkt->stream_index != self->video_idx) {
            av_packet_unref(self->pkt);
            continue;
        }
        if (!self->got_keyframe) {
            if (!(self->pkt->flags & AV_PKT_FLAG_KEY)) {
                av_packet_unref(self->pkt);
                continue;
            }
            self->got_keyframe = 1;
        }
        ret = avcodec_send_packet(self->dec, self->pkt);
        av_packet_unref(self->pkt);
        if (ret < 0 && ret != AVERROR(EAGAIN)) {
            continue;
        }
        ret = avcodec_receive_frame(self->dec, self->frame);
        if (ret == 0) {
            got_frame = 1;
            width = self->frame->width;
            height = self->frame->height;
            pix_fmt = self->frame->format;
            break;
        }
    }
    Py_END_ALLOW_THREADS

    if (!got_frame) {
        return Py_BuildValue("(OOii)", Py_False, Py_None, 0, 0);
    }

    if (!convert) {
        av_frame_unref(self->frame);
        return Py_BuildValue("(OOii)", Py_True, Py_None, width, height);
    }

    Py_ssize_t nbytes = (Py_ssize_t)width * (Py_ssize_t)height * 3;
    PyObject *ba = PyByteArray_FromStringAndSize(NULL, nbytes);
    if (!ba) {
        av_frame_unref(self->frame);
        return NULL;
    }
    uint8_t *dst_ptr = (uint8_t *)PyByteArray_AS_STRING(ba);
    int sws_ok = 1;

    Py_BEGIN_ALLOW_THREADS
    if (!self->sws || self->sws_w != width || self->sws_h != height || self->sws_fmt != pix_fmt) {
        if (self->sws) {
            sws_freeContext(self->sws);
        }
        self->sws = sws_getContext(
            width, height, pix_fmt,
            width, height, AV_PIX_FMT_BGR24,
            SWS_FAST_BILINEAR, NULL, NULL, NULL
        );
        self->sws_w = width;
        self->sws_h = height;
        self->sws_fmt = pix_fmt;
    }
    if (!self->sws) {
        sws_ok = 0;
    } else {
        uint8_t *dst[4] = {dst_ptr, NULL, NULL, NULL};
        int dst_linesize[4] = {width * 3, 0, 0, 0};
        sws_scale(
            self->sws,
            (const uint8_t *const *)self->frame->data, self->frame->linesize,
            0, height, dst, dst_linesize
        );
    }
    av_frame_unref(self->frame);
    Py_END_ALLOW_THREADS

    if (!sws_ok) {
        Py_DECREF(ba);
        PyErr_SetString(PyExc_RuntimeError, "sws_getContext failed");
        return NULL;
    }

    PyObject *res = Py_BuildValue("(OOii)", Py_True, ba, width, height);
    Py_DECREF(ba);
    return res;
}

static PyObject *FastRtsp_abort(FastRtspObject *self, PyObject *Py_UNUSED(ignored)) {
    self->aborted = 1;
    Py_RETURN_NONE;
}

static PyObject *FastRtsp_close(FastRtspObject *self, PyObject *Py_UNUSED(ignored)) {
    self->aborted = 1;
    free_resources(self);
    Py_RETURN_NONE;
}

static PyMethodDef FastRtsp_methods[] = {
    {"read", (PyCFunction)FastRtsp_read, METH_VARARGS | METH_KEYWORDS,
     "Read one frame. Returns (ok, bytearray|None, width, height)."},
    {"abort", (PyCFunction)FastRtsp_abort, METH_NOARGS,
     "Interrupt any blocking RTSP I/O."},
    {"close", (PyCFunction)FastRtsp_close, METH_NOARGS,
     "Close RTSP stream and free FFmpeg resources."},
    {NULL, NULL, 0, NULL}
};

static PyTypeObject FastRtspType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "onvif_cam._fast_rtsp.FastRtspCapture",
    .tp_doc = "Low-latency single-threaded FFmpeg RTSP video capture",
    .tp_basicsize = sizeof(FastRtspObject),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = PyType_GenericNew,
    .tp_init = (initproc)FastRtsp_init,
    .tp_dealloc = (destructor)FastRtsp_dealloc,
    .tp_methods = FastRtsp_methods,
};

static PyModuleDef fast_rtsp_module = {
    PyModuleDef_HEAD_INIT,
    .m_name = "_fast_rtsp",
    .m_doc = "Low-latency RTSP capture extension using libavformat/libavcodec/libswscale",
    .m_size = -1,
};

PyMODINIT_FUNC PyInit__fast_rtsp(void) {
    av_log_set_level(AV_LOG_ERROR);
    if (PyType_Ready(&FastRtspType) < 0) {
        return NULL;
    }
    PyObject *m = PyModule_Create(&fast_rtsp_module);
    if (!m) {
        return NULL;
    }
    Py_INCREF(&FastRtspType);
    if (PyModule_AddObject(m, "FastRtspCapture", (PyObject *)&FastRtspType) < 0) {
        Py_DECREF(&FastRtspType);
        Py_DECREF(m);
        return NULL;
    }
    return m;
}
