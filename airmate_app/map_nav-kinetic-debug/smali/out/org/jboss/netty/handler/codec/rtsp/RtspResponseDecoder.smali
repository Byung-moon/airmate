.class public Lorg/jboss/netty/handler/codec/rtsp/RtspResponseDecoder;
.super Lorg/jboss/netty/handler/codec/rtsp/RtspMessageDecoder;
.source "RtspResponseDecoder.java"


# direct methods
.method public constructor <init>()V
    .registers 1

    .line 60
    invoke-direct {p0}, Lorg/jboss/netty/handler/codec/rtsp/RtspMessageDecoder;-><init>()V

    .line 61
    return-void
.end method

.method public constructor <init>(III)V
    .registers 4
    .param p1, "maxInitialLineLength"    # I
    .param p2, "maxHeaderSize"    # I
    .param p3, "maxContentLength"    # I

    .line 68
    invoke-direct {p0, p1, p2, p3}, Lorg/jboss/netty/handler/codec/rtsp/RtspMessageDecoder;-><init>(III)V

    .line 69
    return-void
.end method


# virtual methods
.method protected createMessage([Ljava/lang/String;)Lorg/jboss/netty/handler/codec/http/HttpMessage;
    .registers 7
    .param p1, "initialLine"    # [Ljava/lang/String;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/Exception;
        }
    .end annotation

    .line 73
    new-instance v0, Lorg/jboss/netty/handler/codec/http/DefaultHttpResponse;

    const/4 v1, 0x0

    aget-object v1, p1, v1

    invoke-static {v1}, Lorg/jboss/netty/handler/codec/rtsp/RtspVersions;->valueOf(Ljava/lang/String;)Lorg/jboss/netty/handler/codec/http/HttpVersion;

    move-result-object v1

    new-instance v2, Lorg/jboss/netty/handler/codec/http/HttpResponseStatus;

    const/4 v3, 0x1

    aget-object v3, p1, v3

    invoke-static {v3}, Ljava/lang/Integer;->valueOf(Ljava/lang/String;)Ljava/lang/Integer;

    move-result-object v3

    invoke-virtual {v3}, Ljava/lang/Integer;->intValue()I

    move-result v3

    const/4 v4, 0x2

    aget-object v4, p1, v4

    invoke-direct {v2, v3, v4}, Lorg/jboss/netty/handler/codec/http/HttpResponseStatus;-><init>(ILjava/lang/String;)V

    invoke-direct {v0, v1, v2}, Lorg/jboss/netty/handler/codec/http/DefaultHttpResponse;-><init>(Lorg/jboss/netty/handler/codec/http/HttpVersion;Lorg/jboss/netty/handler/codec/http/HttpResponseStatus;)V

    return-object v0
.end method

.method protected isDecodingRequest()Z
    .registers 2

    .line 80
    const/4 v0, 0x0

    return v0
.end method
