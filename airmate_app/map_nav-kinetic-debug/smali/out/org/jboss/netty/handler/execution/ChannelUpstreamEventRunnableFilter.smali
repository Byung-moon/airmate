.class public Lorg/jboss/netty/handler/execution/ChannelUpstreamEventRunnableFilter;
.super Ljava/lang/Object;
.source "ChannelUpstreamEventRunnableFilter.java"

# interfaces
.implements Lorg/jboss/netty/handler/execution/ChannelEventRunnableFilter;


# direct methods
.method public constructor <init>()V
    .registers 1

    .line 21
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public filter(Lorg/jboss/netty/handler/execution/ChannelEventRunnable;)Z
    .registers 3
    .param p1, "event"    # Lorg/jboss/netty/handler/execution/ChannelEventRunnable;

    .line 23
    instance-of v0, p1, Lorg/jboss/netty/handler/execution/ChannelDownstreamEventRunnable;

    return v0
.end method