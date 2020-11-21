.class Lorg/jboss/netty/logging/OsgiLogger;
.super Lorg/jboss/netty/logging/AbstractInternalLogger;
.source "OsgiLogger.java"


# instance fields
.field private final fallback:Lorg/jboss/netty/logging/InternalLogger;

.field private final name:Ljava/lang/String;

.field private final parent:Lorg/jboss/netty/logging/OsgiLoggerFactory;

.field private final prefix:Ljava/lang/String;


# direct methods
.method constructor <init>(Lorg/jboss/netty/logging/OsgiLoggerFactory;Ljava/lang/String;Lorg/jboss/netty/logging/InternalLogger;)V
    .registers 6
    .param p1, "parent"    # Lorg/jboss/netty/logging/OsgiLoggerFactory;
    .param p2, "name"    # Ljava/lang/String;
    .param p3, "fallback"    # Lorg/jboss/netty/logging/InternalLogger;

    .line 30
    invoke-direct {p0}, Lorg/jboss/netty/logging/AbstractInternalLogger;-><init>()V

    .line 31
    iput-object p1, p0, Lorg/jboss/netty/logging/OsgiLogger;->parent:Lorg/jboss/netty/logging/OsgiLoggerFactory;

    .line 32
    iput-object p2, p0, Lorg/jboss/netty/logging/OsgiLogger;->name:Ljava/lang/String;

    .line 33
    iput-object p3, p0, Lorg/jboss/netty/logging/OsgiLogger;->fallback:Lorg/jboss/netty/logging/InternalLogger;

    .line 34
    new-instance v0, Ljava/lang/StringBuilder;

    invoke-direct {v0}, Ljava/lang/StringBuilder;-><init>()V

    const-string v1, "["

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v0, p2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v1, "] "

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v0}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v0

    iput-object v0, p0, Lorg/jboss/netty/logging/OsgiLogger;->prefix:Ljava/lang/String;

    .line 35
    return-void
.end method


# virtual methods
.method public debug(Ljava/lang/String;)V
    .registers 6
    .param p1, "msg"    # Ljava/lang/String;

    .line 38
    iget-object v0, p0, Lorg/jboss/netty/logging/OsgiLogger;->parent:Lorg/jboss/netty/logging/OsgiLoggerFactory;

    invoke-virtual {v0}, Lorg/jboss/netty/logging/OsgiLoggerFactory;->getLogService()Lorg/osgi/service/log/LogService;

    move-result-object v0

    .line 39
    .local v0, "logService":Lorg/osgi/service/log/LogService;
    if-eqz v0, :cond_1e

    .line 40
    const/4 v1, 0x4

    new-instance v2, Ljava/lang/StringBuilder;

    invoke-direct {v2}, Ljava/lang/StringBuilder;-><init>()V

    iget-object v3, p0, Lorg/jboss/netty/logging/OsgiLogger;->prefix:Ljava/lang/String;

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2, p1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-interface {v0, v1, v2}, Lorg/osgi/service/log/LogService;->log(ILjava/lang/String;)V

    goto :goto_23

    .line 42
    :cond_1e
    iget-object v1, p0, Lorg/jboss/netty/logging/OsgiLogger;->fallback:Lorg/jboss/netty/logging/InternalLogger;

    invoke-interface {v1, p1}, Lorg/jboss/netty/logging/InternalLogger;->debug(Ljava/lang/String;)V

    .line 44
    :goto_23
    return-void
.end method

.method public debug(Ljava/lang/String;Ljava/lang/Throwable;)V
    .registers 7
    .param p1, "msg"    # Ljava/lang/String;
    .param p2, "cause"    # Ljava/lang/Throwable;

    .line 47
    iget-object v0, p0, Lorg/jboss/netty/logging/OsgiLogger;->parent:Lorg/jboss/netty/logging/OsgiLoggerFactory;

    invoke-virtual {v0}, Lorg/jboss/netty/logging/OsgiLoggerFactory;->getLogService()Lorg/osgi/service/log/LogService;

    move-result-object v0

    .line 48
    .local v0, "logService":Lorg/osgi/service/log/LogService;
    if-eqz v0, :cond_1e

    .line 49
    const/4 v1, 0x4

    new-instance v2, Ljava/lang/StringBuilder;

    invoke-direct {v2}, Ljava/lang/StringBuilder;-><init>()V

    iget-object v3, p0, Lorg/jboss/netty/logging/OsgiLogger;->prefix:Ljava/lang/String;

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2, p1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-interface {v0, v1, v2, p2}, Lorg/osgi/service/log/LogService;->log(ILjava/lang/String;Ljava/lang/Throwable;)V

    goto :goto_23

    .line 51
    :cond_1e
    iget-object v1, p0, Lorg/jboss/netty/logging/OsgiLogger;->fallback:Lorg/jboss/netty/logging/InternalLogger;

    invoke-interface {v1, p1, p2}, Lorg/jboss/netty/logging/InternalLogger;->debug(Ljava/lang/String;Ljava/lang/Throwable;)V

    .line 53
    :goto_23
    return-void
.end method

.method public error(Ljava/lang/String;)V
    .registers 6
    .param p1, "msg"    # Ljava/lang/String;

    .line 56
    iget-object v0, p0, Lorg/jboss/netty/logging/OsgiLogger;->parent:Lorg/jboss/netty/logging/OsgiLoggerFactory;

    invoke-virtual {v0}, Lorg/jboss/netty/logging/OsgiLoggerFactory;->getLogService()Lorg/osgi/service/log/LogService;

    move-result-object v0

    .line 57
    .local v0, "logService":Lorg/osgi/service/log/LogService;
    if-eqz v0, :cond_1e

    .line 58
    const/4 v1, 0x1

    new-instance v2, Ljava/lang/StringBuilder;

    invoke-direct {v2}, Ljava/lang/StringBuilder;-><init>()V

    iget-object v3, p0, Lorg/jboss/netty/logging/OsgiLogger;->prefix:Ljava/lang/String;

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2, p1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-interface {v0, v1, v2}, Lorg/osgi/service/log/LogService;->log(ILjava/lang/String;)V

    goto :goto_23

    .line 60
    :cond_1e
    iget-object v1, p0, Lorg/jboss/netty/logging/OsgiLogger;->fallback:Lorg/jboss/netty/logging/InternalLogger;

    invoke-interface {v1, p1}, Lorg/jboss/netty/logging/InternalLogger;->error(Ljava/lang/String;)V

    .line 62
    :goto_23
    return-void
.end method

.method public error(Ljava/lang/String;Ljava/lang/Throwable;)V
    .registers 7
    .param p1, "msg"    # Ljava/lang/String;
    .param p2, "cause"    # Ljava/lang/Throwable;

    .line 65
    iget-object v0, p0, Lorg/jboss/netty/logging/OsgiLogger;->parent:Lorg/jboss/netty/logging/OsgiLoggerFactory;

    invoke-virtual {v0}, Lorg/jboss/netty/logging/OsgiLoggerFactory;->getLogService()Lorg/osgi/service/log/LogService;

    move-result-object v0

    .line 66
    .local v0, "logService":Lorg/osgi/service/log/LogService;
    if-eqz v0, :cond_1e

    .line 67
    const/4 v1, 0x1

    new-instance v2, Ljava/lang/StringBuilder;

    invoke-direct {v2}, Ljava/lang/StringBuilder;-><init>()V

    iget-object v3, p0, Lorg/jboss/netty/logging/OsgiLogger;->prefix:Ljava/lang/String;

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2, p1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-interface {v0, v1, v2, p2}, Lorg/osgi/service/log/LogService;->log(ILjava/lang/String;Ljava/lang/Throwable;)V

    goto :goto_23

    .line 69
    :cond_1e
    iget-object v1, p0, Lorg/jboss/netty/logging/OsgiLogger;->fallback:Lorg/jboss/netty/logging/InternalLogger;

    invoke-interface {v1, p1, p2}, Lorg/jboss/netty/logging/InternalLogger;->error(Ljava/lang/String;Ljava/lang/Throwable;)V

    .line 71
    :goto_23
    return-void
.end method

.method public info(Ljava/lang/String;)V
    .registers 6
    .param p1, "msg"    # Ljava/lang/String;

    .line 74
    iget-object v0, p0, Lorg/jboss/netty/logging/OsgiLogger;->parent:Lorg/jboss/netty/logging/OsgiLoggerFactory;

    invoke-virtual {v0}, Lorg/jboss/netty/logging/OsgiLoggerFactory;->getLogService()Lorg/osgi/service/log/LogService;

    move-result-object v0

    .line 75
    .local v0, "logService":Lorg/osgi/service/log/LogService;
    if-eqz v0, :cond_1e

    .line 76
    const/4 v1, 0x3

    new-instance v2, Ljava/lang/StringBuilder;

    invoke-direct {v2}, Ljava/lang/StringBuilder;-><init>()V

    iget-object v3, p0, Lorg/jboss/netty/logging/OsgiLogger;->prefix:Ljava/lang/String;

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2, p1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-interface {v0, v1, v2}, Lorg/osgi/service/log/LogService;->log(ILjava/lang/String;)V

    goto :goto_23

    .line 78
    :cond_1e
    iget-object v1, p0, Lorg/jboss/netty/logging/OsgiLogger;->fallback:Lorg/jboss/netty/logging/InternalLogger;

    invoke-interface {v1, p1}, Lorg/jboss/netty/logging/InternalLogger;->info(Ljava/lang/String;)V

    .line 80
    :goto_23
    return-void
.end method

.method public info(Ljava/lang/String;Ljava/lang/Throwable;)V
    .registers 7
    .param p1, "msg"    # Ljava/lang/String;
    .param p2, "cause"    # Ljava/lang/Throwable;

    .line 83
    iget-object v0, p0, Lorg/jboss/netty/logging/OsgiLogger;->parent:Lorg/jboss/netty/logging/OsgiLoggerFactory;

    invoke-virtual {v0}, Lorg/jboss/netty/logging/OsgiLoggerFactory;->getLogService()Lorg/osgi/service/log/LogService;

    move-result-object v0

    .line 84
    .local v0, "logService":Lorg/osgi/service/log/LogService;
    if-eqz v0, :cond_1e

    .line 85
    const/4 v1, 0x3

    new-instance v2, Ljava/lang/StringBuilder;

    invoke-direct {v2}, Ljava/lang/StringBuilder;-><init>()V

    iget-object v3, p0, Lorg/jboss/netty/logging/OsgiLogger;->prefix:Ljava/lang/String;

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2, p1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-interface {v0, v1, v2, p2}, Lorg/osgi/service/log/LogService;->log(ILjava/lang/String;Ljava/lang/Throwable;)V

    goto :goto_23

    .line 87
    :cond_1e
    iget-object v1, p0, Lorg/jboss/netty/logging/OsgiLogger;->fallback:Lorg/jboss/netty/logging/InternalLogger;

    invoke-interface {v1, p1, p2}, Lorg/jboss/netty/logging/InternalLogger;->info(Ljava/lang/String;Ljava/lang/Throwable;)V

    .line 89
    :goto_23
    return-void
.end method

.method public isDebugEnabled()Z
    .registers 2

    .line 92
    const/4 v0, 0x1

    return v0
.end method

.method public isErrorEnabled()Z
    .registers 2

    .line 96
    const/4 v0, 0x1

    return v0
.end method

.method public isInfoEnabled()Z
    .registers 2

    .line 100
    const/4 v0, 0x1

    return v0
.end method

.method public isWarnEnabled()Z
    .registers 2

    .line 104
    const/4 v0, 0x1

    return v0
.end method

.method public toString()Ljava/lang/String;
    .registers 2

    .line 127
    iget-object v0, p0, Lorg/jboss/netty/logging/OsgiLogger;->name:Ljava/lang/String;

    return-object v0
.end method

.method public warn(Ljava/lang/String;)V
    .registers 6
    .param p1, "msg"    # Ljava/lang/String;

    .line 108
    iget-object v0, p0, Lorg/jboss/netty/logging/OsgiLogger;->parent:Lorg/jboss/netty/logging/OsgiLoggerFactory;

    invoke-virtual {v0}, Lorg/jboss/netty/logging/OsgiLoggerFactory;->getLogService()Lorg/osgi/service/log/LogService;

    move-result-object v0

    .line 109
    .local v0, "logService":Lorg/osgi/service/log/LogService;
    if-eqz v0, :cond_1e

    .line 110
    const/4 v1, 0x2

    new-instance v2, Ljava/lang/StringBuilder;

    invoke-direct {v2}, Ljava/lang/StringBuilder;-><init>()V

    iget-object v3, p0, Lorg/jboss/netty/logging/OsgiLogger;->prefix:Ljava/lang/String;

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2, p1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-interface {v0, v1, v2}, Lorg/osgi/service/log/LogService;->log(ILjava/lang/String;)V

    goto :goto_23

    .line 112
    :cond_1e
    iget-object v1, p0, Lorg/jboss/netty/logging/OsgiLogger;->fallback:Lorg/jboss/netty/logging/InternalLogger;

    invoke-interface {v1, p1}, Lorg/jboss/netty/logging/InternalLogger;->warn(Ljava/lang/String;)V

    .line 114
    :goto_23
    return-void
.end method

.method public warn(Ljava/lang/String;Ljava/lang/Throwable;)V
    .registers 7
    .param p1, "msg"    # Ljava/lang/String;
    .param p2, "cause"    # Ljava/lang/Throwable;

    .line 117
    iget-object v0, p0, Lorg/jboss/netty/logging/OsgiLogger;->parent:Lorg/jboss/netty/logging/OsgiLoggerFactory;

    invoke-virtual {v0}, Lorg/jboss/netty/logging/OsgiLoggerFactory;->getLogService()Lorg/osgi/service/log/LogService;

    move-result-object v0

    .line 118
    .local v0, "logService":Lorg/osgi/service/log/LogService;
    if-eqz v0, :cond_1e

    .line 119
    const/4 v1, 0x2

    new-instance v2, Ljava/lang/StringBuilder;

    invoke-direct {v2}, Ljava/lang/StringBuilder;-><init>()V

    iget-object v3, p0, Lorg/jboss/netty/logging/OsgiLogger;->prefix:Ljava/lang/String;

    invoke-virtual {v2, v3}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2, p1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v2}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    invoke-interface {v0, v1, v2, p2}, Lorg/osgi/service/log/LogService;->log(ILjava/lang/String;Ljava/lang/Throwable;)V

    goto :goto_23

    .line 121
    :cond_1e
    iget-object v1, p0, Lorg/jboss/netty/logging/OsgiLogger;->fallback:Lorg/jboss/netty/logging/InternalLogger;

    invoke-interface {v1, p1, p2}, Lorg/jboss/netty/logging/InternalLogger;->warn(Ljava/lang/String;Ljava/lang/Throwable;)V

    .line 123
    :goto_23
    return-void
.end method
