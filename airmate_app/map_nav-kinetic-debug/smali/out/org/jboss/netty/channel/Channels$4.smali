.class final Lorg/jboss/netty/channel/Channels$4;
.super Ljava/lang/Object;
.source "Channels.java"

# interfaces
.implements Ljava/lang/Runnable;


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Lorg/jboss/netty/channel/Channels;->fireChannelDisconnectedLater(Lorg/jboss/netty/channel/Channel;)Lorg/jboss/netty/channel/ChannelFuture;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x8
    name = null
.end annotation


# instance fields
.field final synthetic val$channel:Lorg/jboss/netty/channel/Channel;


# direct methods
.method constructor <init>(Lorg/jboss/netty/channel/Channel;)V
    .registers 2

    .line 386
    iput-object p1, p0, Lorg/jboss/netty/channel/Channels$4;->val$channel:Lorg/jboss/netty/channel/Channel;

    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public run()V
    .registers 2

    .line 389
    iget-object v0, p0, Lorg/jboss/netty/channel/Channels$4;->val$channel:Lorg/jboss/netty/channel/Channel;

    invoke-static {v0}, Lorg/jboss/netty/channel/Channels;->fireChannelDisconnected(Lorg/jboss/netty/channel/Channel;)V

    .line 390
    return-void
.end method
