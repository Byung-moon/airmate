.class public Lorg/jboss/netty/logging/Log4JLoggerFactory;
.super Lorg/jboss/netty/logging/InternalLoggerFactory;
.source "Log4JLoggerFactory.java"


# direct methods
.method public constructor <init>()V
    .registers 1

    .line 23
    invoke-direct {p0}, Lorg/jboss/netty/logging/InternalLoggerFactory;-><init>()V

    return-void
.end method


# virtual methods
.method public newInstance(Ljava/lang/String;)Lorg/jboss/netty/logging/InternalLogger;
    .registers 4
    .param p1, "name"    # Ljava/lang/String;

    .line 27
    invoke-static {p1}, Lorg/apache/log4j/Logger;->getLogger(Ljava/lang/String;)Lorg/apache/log4j/Logger;

    move-result-object v0

    .line 29
    .local v0, "logger":Lorg/apache/log4j/Logger;
    new-instance v1, Lorg/jboss/netty/logging/Log4JLogger;

    invoke-direct {v1, v0}, Lorg/jboss/netty/logging/Log4JLogger;-><init>(Lorg/apache/log4j/Logger;)V

    return-object v1
.end method
