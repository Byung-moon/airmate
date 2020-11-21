.class public abstract Lorg/jboss/netty/buffer/AbstractChannelBufferFactory;
.super Ljava/lang/Object;
.source "AbstractChannelBufferFactory.java"

# interfaces
.implements Lorg/jboss/netty/buffer/ChannelBufferFactory;


# instance fields
.field private final defaultOrder:Ljava/nio/ByteOrder;


# direct methods
.method protected constructor <init>()V
    .registers 2

    .line 32
    sget-object v0, Ljava/nio/ByteOrder;->BIG_ENDIAN:Ljava/nio/ByteOrder;

    invoke-direct {p0, v0}, Lorg/jboss/netty/buffer/AbstractChannelBufferFactory;-><init>(Ljava/nio/ByteOrder;)V

    .line 33
    return-void
.end method

.method protected constructor <init>(Ljava/nio/ByteOrder;)V
    .registers 4
    .param p1, "defaultOrder"    # Ljava/nio/ByteOrder;

    .line 40
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 41
    if-eqz p1, :cond_8

    .line 44
    iput-object p1, p0, Lorg/jboss/netty/buffer/AbstractChannelBufferFactory;->defaultOrder:Ljava/nio/ByteOrder;

    .line 45
    return-void

    .line 42
    :cond_8
    new-instance v0, Ljava/lang/NullPointerException;

    const-string v1, "defaultOrder"

    invoke-direct {v0, v1}, Ljava/lang/NullPointerException;-><init>(Ljava/lang/String;)V

    throw v0
.end method


# virtual methods
.method public getBuffer(I)Lorg/jboss/netty/buffer/ChannelBuffer;
    .registers 3
    .param p1, "capacity"    # I

    .line 48
    invoke-virtual {p0}, Lorg/jboss/netty/buffer/AbstractChannelBufferFactory;->getDefaultOrder()Ljava/nio/ByteOrder;

    move-result-object v0

    invoke-virtual {p0, v0, p1}, Lorg/jboss/netty/buffer/AbstractChannelBufferFactory;->getBuffer(Ljava/nio/ByteOrder;I)Lorg/jboss/netty/buffer/ChannelBuffer;

    move-result-object v0

    return-object v0
.end method

.method public getBuffer([BII)Lorg/jboss/netty/buffer/ChannelBuffer;
    .registers 5
    .param p1, "array"    # [B
    .param p2, "offset"    # I
    .param p3, "length"    # I

    .line 52
    invoke-virtual {p0}, Lorg/jboss/netty/buffer/AbstractChannelBufferFactory;->getDefaultOrder()Ljava/nio/ByteOrder;

    move-result-object v0

    invoke-virtual {p0, v0, p1, p2, p3}, Lorg/jboss/netty/buffer/AbstractChannelBufferFactory;->getBuffer(Ljava/nio/ByteOrder;[BII)Lorg/jboss/netty/buffer/ChannelBuffer;

    move-result-object v0

    return-object v0
.end method

.method public getDefaultOrder()Ljava/nio/ByteOrder;
    .registers 2

    .line 56
    iget-object v0, p0, Lorg/jboss/netty/buffer/AbstractChannelBufferFactory;->defaultOrder:Ljava/nio/ByteOrder;

    return-object v0
.end method
