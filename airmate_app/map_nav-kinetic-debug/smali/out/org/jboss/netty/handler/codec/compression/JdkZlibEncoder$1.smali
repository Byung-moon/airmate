.class Lorg/jboss/netty/handler/codec/compression/JdkZlibEncoder$1;
.super Ljava/lang/Object;
.source "JdkZlibEncoder.java"

# interfaces
.implements Lorg/jboss/netty/channel/ChannelFutureListener;


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Lorg/jboss/netty/handler/codec/compression/JdkZlibEncoder;->finishEncode(Lorg/jboss/netty/channel/ChannelHandlerContext;Lorg/jboss/netty/channel/ChannelEvent;)Lorg/jboss/netty/channel/ChannelFuture;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x0
    name = null
.end annotation


# instance fields
.field final synthetic this$0:Lorg/jboss/netty/handler/codec/compression/JdkZlibEncoder;

.field final synthetic val$ctx:Lorg/jboss/netty/channel/ChannelHandlerContext;

.field final synthetic val$evt:Lorg/jboss/netty/channel/ChannelEvent;


# direct methods
.method constructor <init>(Lorg/jboss/netty/handler/codec/compression/JdkZlibEncoder;Lorg/jboss/netty/channel/ChannelHandlerContext;Lorg/jboss/netty/channel/ChannelEvent;)V
    .registers 4

    .line 263
    iput-object p1, p0, Lorg/jboss/netty/handler/codec/compression/JdkZlibEncoder$1;->this$0:Lorg/jboss/netty/handler/codec/compression/JdkZlibEncoder;

    iput-object p2, p0, Lorg/jboss/netty/handler/codec/compression/JdkZlibEncoder$1;->val$ctx:Lorg/jboss/netty/channel/ChannelHandlerContext;

    iput-object p3, p0, Lorg/jboss/netty/handler/codec/compression/JdkZlibEncoder$1;->val$evt:Lorg/jboss/netty/channel/ChannelEvent;

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

    .line 265
    iget-object v0, p0, Lorg/jboss/netty/handler/codec/compression/JdkZlibEncoder$1;->val$ctx:Lorg/jboss/netty/channel/ChannelHandlerContext;

    iget-object v1, p0, Lorg/jboss/netty/handler/codec/compression/JdkZlibEncoder$1;->val$evt:Lorg/jboss/netty/channel/ChannelEvent;

    invoke-interface {v0, v1}, Lorg/jboss/netty/channel/ChannelHandlerContext;->sendDownstream(Lorg/jboss/netty/channel/ChannelEvent;)V

    .line 266
    return-void
.end method
