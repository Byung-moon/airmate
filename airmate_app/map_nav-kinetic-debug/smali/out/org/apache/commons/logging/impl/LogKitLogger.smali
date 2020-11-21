.class public Lorg/apache/commons/logging/impl/LogKitLogger;
.super Ljava/lang/Object;
.source "LogKitLogger.java"

# interfaces
.implements Lorg/apache/commons/logging/Log;
.implements Ljava/io/Serializable;


# instance fields
.field protected transient logger:Lorg/apache/log/Logger;

.field protected name:Ljava/lang/String;


# direct methods
.method public constructor <init>(Ljava/lang/String;)V
    .registers 3
    .param p1, "name"    # Ljava/lang/String;

    .line 63
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 48
    const/4 v0, 0x0

    iput-object v0, p0, Lorg/apache/commons/logging/impl/LogKitLogger;->logger:Lorg/apache/log/Logger;

    .line 51
    iput-object v0, p0, Lorg/apache/commons/logging/impl/LogKitLogger;->name:Ljava/lang/String;

    .line 64
    iput-object p1, p0, Lorg/apache/commons/logging/impl/LogKitLogger;->name:Ljava/lang/String;

    .line 65
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    iput-object v0, p0, Lorg/apache/commons/logging/impl/LogKitLogger;->logger:Lorg/apache/log/Logger;

    .line 66
    return-void
.end method


# virtual methods
.method public debug(Ljava/lang/Object;)V
    .registers 4
    .param p1, "message"    # Ljava/lang/Object;

    .line 118
    if-eqz p1, :cond_d

    .line 119
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-static {p1}, Ljava/lang/String;->valueOf(Ljava/lang/Object;)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Lorg/apache/log/Logger;->debug(Ljava/lang/String;)V

    .line 121
    :cond_d
    return-void
.end method

.method public debug(Ljava/lang/Object;Ljava/lang/Throwable;)V
    .registers 5
    .param p1, "message"    # Ljava/lang/Object;
    .param p2, "t"    # Ljava/lang/Throwable;

    .line 132
    if-eqz p1, :cond_d

    .line 133
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-static {p1}, Ljava/lang/String;->valueOf(Ljava/lang/Object;)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1, p2}, Lorg/apache/log/Logger;->debug(Ljava/lang/String;Ljava/lang/Throwable;)V

    .line 135
    :cond_d
    return-void
.end method

.method public error(Ljava/lang/Object;)V
    .registers 4
    .param p1, "message"    # Ljava/lang/Object;

    .line 199
    if-eqz p1, :cond_d

    .line 200
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-static {p1}, Ljava/lang/String;->valueOf(Ljava/lang/Object;)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Lorg/apache/log/Logger;->error(Ljava/lang/String;)V

    .line 202
    :cond_d
    return-void
.end method

.method public error(Ljava/lang/Object;Ljava/lang/Throwable;)V
    .registers 5
    .param p1, "message"    # Ljava/lang/Object;
    .param p2, "t"    # Ljava/lang/Throwable;

    .line 213
    if-eqz p1, :cond_d

    .line 214
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-static {p1}, Ljava/lang/String;->valueOf(Ljava/lang/Object;)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1, p2}, Lorg/apache/log/Logger;->error(Ljava/lang/String;Ljava/lang/Throwable;)V

    .line 216
    :cond_d
    return-void
.end method

.method public fatal(Ljava/lang/Object;)V
    .registers 4
    .param p1, "message"    # Ljava/lang/Object;

    .line 226
    if-eqz p1, :cond_d

    .line 227
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-static {p1}, Ljava/lang/String;->valueOf(Ljava/lang/Object;)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Lorg/apache/log/Logger;->fatalError(Ljava/lang/String;)V

    .line 229
    :cond_d
    return-void
.end method

.method public fatal(Ljava/lang/Object;Ljava/lang/Throwable;)V
    .registers 5
    .param p1, "message"    # Ljava/lang/Object;
    .param p2, "t"    # Ljava/lang/Throwable;

    .line 240
    if-eqz p1, :cond_d

    .line 241
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-static {p1}, Ljava/lang/String;->valueOf(Ljava/lang/Object;)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1, p2}, Lorg/apache/log/Logger;->fatalError(Ljava/lang/String;Ljava/lang/Throwable;)V

    .line 243
    :cond_d
    return-void
.end method

.method public getLogger()Lorg/apache/log/Logger;
    .registers 3

    .line 77
    iget-object v0, p0, Lorg/apache/commons/logging/impl/LogKitLogger;->logger:Lorg/apache/log/Logger;

    if-nez v0, :cond_10

    .line 78
    invoke-static {}, Lorg/apache/log/Hierarchy;->getDefaultHierarchy()Lorg/apache/log/Hierarchy;

    move-result-object v0

    iget-object v1, p0, Lorg/apache/commons/logging/impl/LogKitLogger;->name:Ljava/lang/String;

    invoke-virtual {v0, v1}, Lorg/apache/log/Hierarchy;->getLoggerFor(Ljava/lang/String;)Lorg/apache/log/Logger;

    move-result-object v0

    iput-object v0, p0, Lorg/apache/commons/logging/impl/LogKitLogger;->logger:Lorg/apache/log/Logger;

    .line 80
    :cond_10
    iget-object v0, p0, Lorg/apache/commons/logging/impl/LogKitLogger;->logger:Lorg/apache/log/Logger;

    return-object v0
.end method

.method public info(Ljava/lang/Object;)V
    .registers 4
    .param p1, "message"    # Ljava/lang/Object;

    .line 145
    if-eqz p1, :cond_d

    .line 146
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-static {p1}, Ljava/lang/String;->valueOf(Ljava/lang/Object;)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Lorg/apache/log/Logger;->info(Ljava/lang/String;)V

    .line 148
    :cond_d
    return-void
.end method

.method public info(Ljava/lang/Object;Ljava/lang/Throwable;)V
    .registers 5
    .param p1, "message"    # Ljava/lang/Object;
    .param p2, "t"    # Ljava/lang/Throwable;

    .line 159
    if-eqz p1, :cond_d

    .line 160
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-static {p1}, Ljava/lang/String;->valueOf(Ljava/lang/Object;)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1, p2}, Lorg/apache/log/Logger;->info(Ljava/lang/String;Ljava/lang/Throwable;)V

    .line 162
    :cond_d
    return-void
.end method

.method public isDebugEnabled()Z
    .registers 2

    .line 250
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-virtual {v0}, Lorg/apache/log/Logger;->isDebugEnabled()Z

    move-result v0

    return v0
.end method

.method public isErrorEnabled()Z
    .registers 2

    .line 258
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-virtual {v0}, Lorg/apache/log/Logger;->isErrorEnabled()Z

    move-result v0

    return v0
.end method

.method public isFatalEnabled()Z
    .registers 2

    .line 266
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-virtual {v0}, Lorg/apache/log/Logger;->isFatalErrorEnabled()Z

    move-result v0

    return v0
.end method

.method public isInfoEnabled()Z
    .registers 2

    .line 274
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-virtual {v0}, Lorg/apache/log/Logger;->isInfoEnabled()Z

    move-result v0

    return v0
.end method

.method public isTraceEnabled()Z
    .registers 2

    .line 282
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-virtual {v0}, Lorg/apache/log/Logger;->isDebugEnabled()Z

    move-result v0

    return v0
.end method

.method public isWarnEnabled()Z
    .registers 2

    .line 290
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-virtual {v0}, Lorg/apache/log/Logger;->isWarnEnabled()Z

    move-result v0

    return v0
.end method

.method public trace(Ljava/lang/Object;)V
    .registers 2
    .param p1, "message"    # Ljava/lang/Object;

    .line 95
    invoke-virtual {p0, p1}, Lorg/apache/commons/logging/impl/LogKitLogger;->debug(Ljava/lang/Object;)V

    .line 96
    return-void
.end method

.method public trace(Ljava/lang/Object;Ljava/lang/Throwable;)V
    .registers 3
    .param p1, "message"    # Ljava/lang/Object;
    .param p2, "t"    # Ljava/lang/Throwable;

    .line 107
    invoke-virtual {p0, p1, p2}, Lorg/apache/commons/logging/impl/LogKitLogger;->debug(Ljava/lang/Object;Ljava/lang/Throwable;)V

    .line 108
    return-void
.end method

.method public warn(Ljava/lang/Object;)V
    .registers 4
    .param p1, "message"    # Ljava/lang/Object;

    .line 172
    if-eqz p1, :cond_d

    .line 173
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-static {p1}, Ljava/lang/String;->valueOf(Ljava/lang/Object;)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Lorg/apache/log/Logger;->warn(Ljava/lang/String;)V

    .line 175
    :cond_d
    return-void
.end method

.method public warn(Ljava/lang/Object;Ljava/lang/Throwable;)V
    .registers 5
    .param p1, "message"    # Ljava/lang/Object;
    .param p2, "t"    # Ljava/lang/Throwable;

    .line 186
    if-eqz p1, :cond_d

    .line 187
    invoke-virtual {p0}, Lorg/apache/commons/logging/impl/LogKitLogger;->getLogger()Lorg/apache/log/Logger;

    move-result-object v0

    invoke-static {p1}, Ljava/lang/String;->valueOf(Ljava/lang/Object;)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1, p2}, Lorg/apache/log/Logger;->warn(Ljava/lang/String;Ljava/lang/Throwable;)V

    .line 189
    :cond_d
    return-void
.end method
