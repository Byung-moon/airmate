.class final Lorg/jboss/netty/handler/codec/http/websocketx/WebSocketServerHandshaker$1;
.super Ljava/lang/Object;
.source "WebSocketServerHandshaker.java"

# interfaces
.implements Lorg/jboss/netty/channel/ChannelFutureListener;


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lorg/jboss/netty/handler/codec/http/websocketx/WebSocketServerHandshaker;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x8
    name = null
.end annotation


# direct methods
.method constructor <init>()V
    .registers 1

    .line 47
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public operationComplete(Lorg/jboss/netty/channel/ChannelFuture;)V
    .registers 4
    .param p1, "future"    # Lorg/jboss/netty/channel/ChannelFuture;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/Exception;
        }
    .end annotation

    .line 49
    invoke-interface {p1}, Lorg/jboss/netty/channel/ChannelFuture;->isSuccess()Z

    move-result v0

    if-nez v0, :cond_11

    .line 50
    invoke-interface {p1}, Lorg/jboss/netty/channel/ChannelFuture;->getChannel()Lorg/jboss/netty/channel/Channel;

    move-result-object v0

    invoke-interface {p1}, Lorg/jboss/netty/channel/ChannelFuture;->getCause()Ljava/lang/Throwable;

    move-result-object v1

    invoke-static {v0, v1}, Lorg/jboss/netty/channel/Channels;->fireExceptionCaught(Lorg/jboss/netty/channel/Channel;Ljava/lang/Throwable;)V

    .line 52
    :cond_11
    return-void
.end method
