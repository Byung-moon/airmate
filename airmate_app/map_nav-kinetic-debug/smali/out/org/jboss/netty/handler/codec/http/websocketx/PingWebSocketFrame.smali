.class public Lorg/jboss/netty/handler/codec/http/websocketx/PingWebSocketFrame;
.super Lorg/jboss/netty/handler/codec/http/websocketx/WebSocketFrame;
.source "PingWebSocketFrame.java"


# direct methods
.method public constructor <init>()V
    .registers 2

    .line 29
    invoke-direct {p0}, Lorg/jboss/netty/handler/codec/http/websocketx/WebSocketFrame;-><init>()V

    .line 30
    const/4 v0, 0x1

    invoke-virtual {p0, v0}, Lorg/jboss/netty/handler/codec/http/websocketx/PingWebSocketFrame;->setFinalFragment(Z)V

    .line 31
    sget-object v0, Lorg/jboss/netty/buffer/ChannelBuffers;->EMPTY_BUFFER:Lorg/jboss/netty/buffer/ChannelBuffer;

    invoke-virtual {p0, v0}, Lorg/jboss/netty/handler/codec/http/websocketx/PingWebSocketFrame;->setBinaryData(Lorg/jboss/netty/buffer/ChannelBuffer;)V

    .line 32
    return-void
.end method

.method public constructor <init>(Lorg/jboss/netty/buffer/ChannelBuffer;)V
    .registers 2
    .param p1, "binaryData"    # Lorg/jboss/netty/buffer/ChannelBuffer;

    .line 40
    invoke-direct {p0}, Lorg/jboss/netty/handler/codec/http/websocketx/WebSocketFrame;-><init>()V

    .line 41
    invoke-virtual {p0, p1}, Lorg/jboss/netty/handler/codec/http/websocketx/PingWebSocketFrame;->setBinaryData(Lorg/jboss/netty/buffer/ChannelBuffer;)V

    .line 42
    return-void
.end method

.method public constructor <init>(ZILorg/jboss/netty/buffer/ChannelBuffer;)V
    .registers 4
    .param p1, "finalFragment"    # Z
    .param p2, "rsv"    # I
    .param p3, "binaryData"    # Lorg/jboss/netty/buffer/ChannelBuffer;

    .line 54
    invoke-direct {p0}, Lorg/jboss/netty/handler/codec/http/websocketx/WebSocketFrame;-><init>()V

    .line 55
    invoke-virtual {p0, p1}, Lorg/jboss/netty/handler/codec/http/websocketx/PingWebSocketFrame;->setFinalFragment(Z)V

    .line 56
    invoke-virtual {p0, p2}, Lorg/jboss/netty/handler/codec/http/websocketx/PingWebSocketFrame;->setRsv(I)V

    .line 57
    invoke-virtual {p0, p3}, Lorg/jboss/netty/handler/codec/http/websocketx/PingWebSocketFrame;->setBinaryData(Lorg/jboss/netty/buffer/ChannelBuffer;)V

    .line 58
    return-void
.end method


# virtual methods
.method public toString()Ljava/lang/String;
    .registers 3

    .line 62
    new-instance v0, Ljava/lang/StringBuilder;

    invoke-direct {v0}, Ljava/lang/StringBuilder;-><init>()V

    invoke-virtual {p0}, Ljava/lang/Object;->getClass()Ljava/lang/Class;

    move-result-object v1

    invoke-virtual {v1}, Ljava/lang/Class;->getSimpleName()Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v1, "(data: "

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {p0}, Lorg/jboss/netty/handler/codec/http/websocketx/PingWebSocketFrame;->getBinaryData()Lorg/jboss/netty/buffer/ChannelBuffer;

    move-result-object v1

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/Object;)Ljava/lang/StringBuilder;

    const/16 v1, 0x29

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(C)Ljava/lang/StringBuilder;

    invoke-virtual {v0}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method
