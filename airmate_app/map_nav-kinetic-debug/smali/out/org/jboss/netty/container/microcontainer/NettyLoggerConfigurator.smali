.class public Lorg/jboss/netty/container/microcontainer/NettyLoggerConfigurator;
.super Ljava/lang/Object;
.source "NettyLoggerConfigurator.java"


# direct methods
.method public constructor <init>()V
    .registers 2

    .line 25
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 26
    new-instance v0, Lorg/jboss/netty/logging/JBossLoggerFactory;

    invoke-direct {v0}, Lorg/jboss/netty/logging/JBossLoggerFactory;-><init>()V

    invoke-static {v0}, Lorg/jboss/netty/logging/InternalLoggerFactory;->setDefaultFactory(Lorg/jboss/netty/logging/InternalLoggerFactory;)V

    .line 27
    return-void
.end method
