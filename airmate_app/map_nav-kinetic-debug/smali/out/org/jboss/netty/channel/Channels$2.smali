.class final Lorg/jboss/netty/channel/Channels$2;
.super Ljava/lang/Object;
.source "Channels.java"

# interfaces
.implements Ljava/lang/Runnable;


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Lorg/jboss/netty/channel/Channels;->fireWriteCompleteLater(Lorg/jboss/netty/channel/Channel;J)Lorg/jboss/netty/channel/ChannelFuture;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x8
    name = null
.end annotation


# instance fields
.field final synthetic val$amount:J

.field final synthetic val$channel:Lorg/jboss/netty/channel/Channel;


# direct methods
.method constructor <init>(Lorg/jboss/netty/channel/Channel;J)V
    .registers 4

    .line 306
    iput-object p1, p0, Lorg/jboss/netty/channel/Channels$2;->val$channel:Lorg/jboss/netty/channel/Channel;

    iput-wide p2, p0, Lorg/jboss/netty/channel/Channels$2;->val$amount:J

    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public run()V
    .registers 4

    .line 309
    iget-object v0, p0, Lorg/jboss/netty/channel/Channels$2;->val$channel:Lorg/jboss/netty/channel/Channel;

    iget-wide v1, p0, Lorg/jboss/netty/channel/Channels$2;->val$amount:J

    invoke-static {v0, v1, v2}, Lorg/jboss/netty/channel/Channels;->fireWriteComplete(Lorg/jboss/netty/channel/Channel;J)V

    .line 310
    return-void
.end method
