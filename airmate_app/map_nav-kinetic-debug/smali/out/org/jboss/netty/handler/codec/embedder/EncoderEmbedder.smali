.class public Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder;
.super Lorg/jboss/netty/handler/codec/embedder/AbstractCodecEmbedder;
.source "EncoderEmbedder.java"


# annotations
.annotation system Ldalvik/annotation/Signature;
    value = {
        "<E:",
        "Ljava/lang/Object;",
        ">",
        "Lorg/jboss/netty/handler/codec/embedder/AbstractCodecEmbedder<",
        "TE;>;"
    }
.end annotation


# direct methods
.method public varargs constructor <init>(Lorg/jboss/netty/buffer/ChannelBufferFactory;[Lorg/jboss/netty/channel/ChannelDownstreamHandler;)V
    .registers 3
    .param p1, "bufferFactory"    # Lorg/jboss/netty/buffer/ChannelBufferFactory;
    .param p2, "handlers"    # [Lorg/jboss/netty/channel/ChannelDownstreamHandler;

    .line 66
    .local p0, "this":Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder;, "Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder<TE;>;"
    invoke-direct {p0, p1, p2}, Lorg/jboss/netty/handler/codec/embedder/AbstractCodecEmbedder;-><init>(Lorg/jboss/netty/buffer/ChannelBufferFactory;[Lorg/jboss/netty/channel/ChannelHandler;)V

    .line 67
    return-void
.end method

.method public varargs constructor <init>([Lorg/jboss/netty/channel/ChannelDownstreamHandler;)V
    .registers 2
    .param p1, "handlers"    # [Lorg/jboss/netty/channel/ChannelDownstreamHandler;

    .line 55
    .local p0, "this":Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder;, "Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder<TE;>;"
    invoke-direct {p0, p1}, Lorg/jboss/netty/handler/codec/embedder/AbstractCodecEmbedder;-><init>([Lorg/jboss/netty/channel/ChannelHandler;)V

    .line 56
    return-void
.end method


# virtual methods
.method public bridge synthetic finish()Z
    .registers 2

    .line 48
    .local p0, "this":Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder;, "Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder<TE;>;"
    invoke-super {p0}, Lorg/jboss/netty/handler/codec/embedder/AbstractCodecEmbedder;->finish()Z

    move-result v0

    return v0
.end method

.method public bridge synthetic getPipeline()Lorg/jboss/netty/channel/ChannelPipeline;
    .registers 2

    .line 48
    .local p0, "this":Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder;, "Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder<TE;>;"
    invoke-super {p0}, Lorg/jboss/netty/handler/codec/embedder/AbstractCodecEmbedder;->getPipeline()Lorg/jboss/netty/channel/ChannelPipeline;

    move-result-object v0

    return-object v0
.end method

.method public offer(Ljava/lang/Object;)Z
    .registers 3
    .param p1, "input"    # Ljava/lang/Object;

    .line 70
    .local p0, "this":Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder;, "Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder<TE;>;"
    invoke-virtual {p0}, Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder;->getChannel()Lorg/jboss/netty/channel/Channel;

    move-result-object v0

    invoke-static {v0, p1}, Lorg/jboss/netty/channel/Channels;->write(Lorg/jboss/netty/channel/Channel;Ljava/lang/Object;)Lorg/jboss/netty/channel/ChannelFuture;

    move-result-object v0

    invoke-interface {v0}, Lorg/jboss/netty/channel/ChannelFuture;->setSuccess()Z

    .line 71
    invoke-virtual {p0}, Lorg/jboss/netty/handler/codec/embedder/EncoderEmbedder;->isEmpty()Z

    move-result v0

    xor-int/lit8 v0, v0, 0x1

    return v0
.end method
