.class public Lorg/jboss/netty/handler/codec/rtsp/RtspRequestEncoder;
.super Lorg/jboss/netty/handler/codec/rtsp/RtspMessageEncoder;
.source "RtspRequestEncoder.java"


# direct methods
.method public constructor <init>()V
    .registers 1

    .line 27
    invoke-direct {p0}, Lorg/jboss/netty/handler/codec/rtsp/RtspMessageEncoder;-><init>()V

    return-void
.end method


# virtual methods
.method protected encodeInitialLine(Lorg/jboss/netty/buffer/ChannelBuffer;Lorg/jboss/netty/handler/codec/http/HttpMessage;)V
    .registers 7
    .param p1, "buf"    # Lorg/jboss/netty/buffer/ChannelBuffer;
    .param p2, "message"    # Lorg/jboss/netty/handler/codec/http/HttpMessage;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/Exception;
        }
    .end annotation

    .line 32
    move-object v0, p2

    check-cast v0, Lorg/jboss/netty/handler/codec/http/HttpRequest;

    .line 33
    .local v0, "request":Lorg/jboss/netty/handler/codec/http/HttpRequest;
    invoke-interface {v0}, Lorg/jboss/netty/handler/codec/http/HttpRequest;->getMethod()Lorg/jboss/netty/handler/codec/http/HttpMethod;

    move-result-object v1

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;->toString()Ljava/lang/String;

    move-result-object v1

    const-string v2, "ASCII"

    invoke-virtual {v1, v2}, Ljava/lang/String;->getBytes(Ljava/lang/String;)[B

    move-result-object v1

    invoke-interface {p1, v1}, Lorg/jboss/netty/buffer/ChannelBuffer;->writeBytes([B)V

    .line 34
    const/16 v1, 0x20

    invoke-interface {p1, v1}, Lorg/jboss/netty/buffer/ChannelBuffer;->writeByte(I)V

    .line 35
    invoke-interface {v0}, Lorg/jboss/netty/handler/codec/http/HttpRequest;->getUri()Ljava/lang/String;

    move-result-object v2

    const-string v3, "ASCII"

    invoke-virtual {v2, v3}, Ljava/lang/String;->getBytes(Ljava/lang/String;)[B

    move-result-object v2

    invoke-interface {p1, v2}, Lorg/jboss/netty/buffer/ChannelBuffer;->writeBytes([B)V

    .line 36
    invoke-interface {p1, v1}, Lorg/jboss/netty/buffer/ChannelBuffer;->writeByte(I)V

    .line 37
    invoke-interface {v0}, Lorg/jboss/netty/handler/codec/http/HttpRequest;->getProtocolVersion()Lorg/jboss/netty/handler/codec/http/HttpVersion;

    move-result-object v1

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpVersion;->toString()Ljava/lang/String;

    move-result-object v1

    const-string v2, "ASCII"

    invoke-virtual {v1, v2}, Ljava/lang/String;->getBytes(Ljava/lang/String;)[B

    move-result-object v1

    invoke-interface {p1, v1}, Lorg/jboss/netty/buffer/ChannelBuffer;->writeBytes([B)V

    .line 38
    const/16 v1, 0xd

    invoke-interface {p1, v1}, Lorg/jboss/netty/buffer/ChannelBuffer;->writeByte(I)V

    .line 39
    const/16 v1, 0xa

    invoke-interface {p1, v1}, Lorg/jboss/netty/buffer/ChannelBuffer;->writeByte(I)V

    .line 40
    return-void
.end method
