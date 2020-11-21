.class public Lorg/jboss/netty/handler/codec/marshalling/CompatibleMarshallingEncoder;
.super Lorg/jboss/netty/handler/codec/oneone/OneToOneEncoder;
.source "CompatibleMarshallingEncoder.java"


# annotations
.annotation runtime Lorg/jboss/netty/channel/ChannelHandler$Sharable;
.end annotation


# instance fields
.field private final provider:Lorg/jboss/netty/handler/codec/marshalling/MarshallerProvider;


# direct methods
.method public constructor <init>(Lorg/jboss/netty/handler/codec/marshalling/MarshallerProvider;)V
    .registers 2
    .param p1, "provider"    # Lorg/jboss/netty/handler/codec/marshalling/MarshallerProvider;

    .line 45
    invoke-direct {p0}, Lorg/jboss/netty/handler/codec/oneone/OneToOneEncoder;-><init>()V

    .line 46
    iput-object p1, p0, Lorg/jboss/netty/handler/codec/marshalling/CompatibleMarshallingEncoder;->provider:Lorg/jboss/netty/handler/codec/marshalling/MarshallerProvider;

    .line 47
    return-void
.end method


# virtual methods
.method protected encode(Lorg/jboss/netty/channel/ChannelHandlerContext;Lorg/jboss/netty/channel/Channel;Ljava/lang/Object;)Ljava/lang/Object;
    .registers 8
    .param p1, "ctx"    # Lorg/jboss/netty/channel/ChannelHandlerContext;
    .param p2, "channel"    # Lorg/jboss/netty/channel/Channel;
    .param p3, "msg"    # Ljava/lang/Object;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/Exception;
        }
    .end annotation

    .line 52
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/marshalling/CompatibleMarshallingEncoder;->provider:Lorg/jboss/netty/handler/codec/marshalling/MarshallerProvider;

    invoke-interface {v0, p1}, Lorg/jboss/netty/handler/codec/marshalling/MarshallerProvider;->getMarshaller(Lorg/jboss/netty/channel/ChannelHandlerContext;)Lorg/jboss/marshalling/Marshaller;

    move-result-object v0

    .line 53
    .local v0, "marshaller":Lorg/jboss/marshalling/Marshaller;
    new-instance v1, Lorg/jboss/netty/handler/codec/marshalling/ChannelBufferByteOutput;

    invoke-interface {p1}, Lorg/jboss/netty/channel/ChannelHandlerContext;->getChannel()Lorg/jboss/netty/channel/Channel;

    move-result-object v2

    invoke-interface {v2}, Lorg/jboss/netty/channel/Channel;->getConfig()Lorg/jboss/netty/channel/ChannelConfig;

    move-result-object v2

    invoke-interface {v2}, Lorg/jboss/netty/channel/ChannelConfig;->getBufferFactory()Lorg/jboss/netty/buffer/ChannelBufferFactory;

    move-result-object v2

    const/16 v3, 0x100

    invoke-direct {v1, v2, v3}, Lorg/jboss/netty/handler/codec/marshalling/ChannelBufferByteOutput;-><init>(Lorg/jboss/netty/buffer/ChannelBufferFactory;I)V

    .line 55
    .local v1, "output":Lorg/jboss/netty/handler/codec/marshalling/ChannelBufferByteOutput;
    invoke-interface {v0, v1}, Lorg/jboss/marshalling/Marshaller;->start(Lorg/jboss/marshalling/ByteOutput;)V

    .line 56
    invoke-interface {v0, p3}, Lorg/jboss/marshalling/Marshaller;->writeObject(Ljava/lang/Object;)V

    .line 57
    invoke-interface {v0}, Lorg/jboss/marshalling/Marshaller;->finish()V

    .line 58
    invoke-interface {v0}, Lorg/jboss/marshalling/Marshaller;->close()V

    .line 60
    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/marshalling/ChannelBufferByteOutput;->getBuffer()Lorg/jboss/netty/buffer/ChannelBuffer;

    move-result-object v2

    return-object v2
.end method
