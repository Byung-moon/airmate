.class public Lorg/jboss/netty/handler/codec/base64/Base64Decoder;
.super Lorg/jboss/netty/handler/codec/oneone/OneToOneDecoder;
.source "Base64Decoder.java"


# annotations
.annotation runtime Lorg/jboss/netty/channel/ChannelHandler$Sharable;
.end annotation


# instance fields
.field private final dialect:Lorg/jboss/netty/handler/codec/base64/Base64Dialect;


# direct methods
.method public constructor <init>()V
    .registers 2

    .line 55
    sget-object v0, Lorg/jboss/netty/handler/codec/base64/Base64Dialect;->STANDARD:Lorg/jboss/netty/handler/codec/base64/Base64Dialect;

    invoke-direct {p0, v0}, Lorg/jboss/netty/handler/codec/base64/Base64Decoder;-><init>(Lorg/jboss/netty/handler/codec/base64/Base64Dialect;)V

    .line 56
    return-void
.end method

.method public constructor <init>(Lorg/jboss/netty/handler/codec/base64/Base64Dialect;)V
    .registers 4
    .param p1, "dialect"    # Lorg/jboss/netty/handler/codec/base64/Base64Dialect;

    .line 58
    invoke-direct {p0}, Lorg/jboss/netty/handler/codec/oneone/OneToOneDecoder;-><init>()V

    .line 59
    if-eqz p1, :cond_8

    .line 62
    iput-object p1, p0, Lorg/jboss/netty/handler/codec/base64/Base64Decoder;->dialect:Lorg/jboss/netty/handler/codec/base64/Base64Dialect;

    .line 63
    return-void

    .line 60
    :cond_8
    new-instance v0, Ljava/lang/NullPointerException;

    const-string v1, "dialect"

    invoke-direct {v0, v1}, Ljava/lang/NullPointerException;-><init>(Ljava/lang/String;)V

    throw v0
.end method


# virtual methods
.method protected decode(Lorg/jboss/netty/channel/ChannelHandlerContext;Lorg/jboss/netty/channel/Channel;Ljava/lang/Object;)Ljava/lang/Object;
    .registers 8
    .param p1, "ctx"    # Lorg/jboss/netty/channel/ChannelHandlerContext;
    .param p2, "channel"    # Lorg/jboss/netty/channel/Channel;
    .param p3, "msg"    # Ljava/lang/Object;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/Exception;
        }
    .end annotation

    .line 68
    instance-of v0, p3, Ljava/lang/String;

    if-eqz v0, :cond_e

    .line 69
    move-object v0, p3

    check-cast v0, Ljava/lang/String;

    sget-object v1, Lorg/jboss/netty/util/CharsetUtil;->US_ASCII:Ljava/nio/charset/Charset;

    invoke-static {v0, v1}, Lorg/jboss/netty/buffer/ChannelBuffers;->copiedBuffer(Ljava/lang/CharSequence;Ljava/nio/charset/Charset;)Lorg/jboss/netty/buffer/ChannelBuffer;

    move-result-object p3

    goto :goto_13

    .line 70
    :cond_e
    instance-of v0, p3, Lorg/jboss/netty/buffer/ChannelBuffer;

    if-nez v0, :cond_13

    .line 71
    return-object p3

    .line 74
    :cond_13
    :goto_13
    move-object v0, p3

    check-cast v0, Lorg/jboss/netty/buffer/ChannelBuffer;

    .line 75
    .local v0, "src":Lorg/jboss/netty/buffer/ChannelBuffer;
    invoke-interface {v0}, Lorg/jboss/netty/buffer/ChannelBuffer;->readerIndex()I

    move-result v1

    invoke-interface {v0}, Lorg/jboss/netty/buffer/ChannelBuffer;->readableBytes()I

    move-result v2

    iget-object v3, p0, Lorg/jboss/netty/handler/codec/base64/Base64Decoder;->dialect:Lorg/jboss/netty/handler/codec/base64/Base64Dialect;

    invoke-static {v0, v1, v2, v3}, Lorg/jboss/netty/handler/codec/base64/Base64;->decode(Lorg/jboss/netty/buffer/ChannelBuffer;IILorg/jboss/netty/handler/codec/base64/Base64Dialect;)Lorg/jboss/netty/buffer/ChannelBuffer;

    move-result-object v1

    return-object v1
.end method
