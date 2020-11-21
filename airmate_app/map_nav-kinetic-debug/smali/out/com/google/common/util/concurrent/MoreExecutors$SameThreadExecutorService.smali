.class Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;
.super Lcom/google/common/util/concurrent/AbstractListeningExecutorService;
.source "MoreExecutors.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/util/concurrent/MoreExecutors;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0xa
    name = "SameThreadExecutorService"
.end annotation


# instance fields
.field private final lock:Ljava/util/concurrent/locks/Lock;

.field private runningTasks:I

.field private shutdown:Z

.field private final termination:Ljava/util/concurrent/locks/Condition;


# direct methods
.method private constructor <init>()V
    .registers 2

    .line 228
    invoke-direct {p0}, Lcom/google/common/util/concurrent/AbstractListeningExecutorService;-><init>()V

    .line 234
    new-instance v0, Ljava/util/concurrent/locks/ReentrantLock;

    invoke-direct {v0}, Ljava/util/concurrent/locks/ReentrantLock;-><init>()V

    iput-object v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    .line 237
    iget-object v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v0}, Ljava/util/concurrent/locks/Lock;->newCondition()Ljava/util/concurrent/locks/Condition;

    move-result-object v0

    iput-object v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->termination:Ljava/util/concurrent/locks/Condition;

    .line 246
    const/4 v0, 0x0

    iput v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->runningTasks:I

    .line 247
    iput-boolean v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->shutdown:Z

    return-void
.end method

.method synthetic constructor <init>(Lcom/google/common/util/concurrent/MoreExecutors$1;)V
    .registers 2
    .param p1, "x0"    # Lcom/google/common/util/concurrent/MoreExecutors$1;

    .line 228
    invoke-direct {p0}, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;-><init>()V

    return-void
.end method

.method private endTask()V
    .registers 3

    .line 339
    iget-object v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v0}, Ljava/util/concurrent/locks/Lock;->lock()V

    .line 341
    :try_start_5
    iget v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->runningTasks:I

    add-int/lit8 v0, v0, -0x1

    iput v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->runningTasks:I

    .line 342
    invoke-virtual {p0}, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->isTerminated()Z

    move-result v0

    if-eqz v0, :cond_16

    .line 343
    iget-object v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->termination:Ljava/util/concurrent/locks/Condition;

    invoke-interface {v0}, Ljava/util/concurrent/locks/Condition;->signalAll()V
    :try_end_16
    .catchall {:try_start_5 .. :try_end_16} :catchall_1d

    .line 346
    :cond_16
    iget-object v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v0}, Ljava/util/concurrent/locks/Lock;->unlock()V

    .line 347
    nop

    .line 348
    return-void

    .line 346
    :catchall_1d
    move-exception v0

    iget-object v1, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v1}, Ljava/util/concurrent/locks/Lock;->unlock()V

    throw v0
.end method

.method private startTask()V
    .registers 3

    .line 324
    iget-object v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v0}, Ljava/util/concurrent/locks/Lock;->lock()V

    .line 326
    :try_start_5
    invoke-virtual {p0}, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->isShutdown()Z

    move-result v0

    if-nez v0, :cond_18

    .line 329
    iget v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->runningTasks:I

    add-int/lit8 v0, v0, 0x1

    iput v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->runningTasks:I
    :try_end_11
    .catchall {:try_start_5 .. :try_end_11} :catchall_20

    .line 331
    iget-object v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v0}, Ljava/util/concurrent/locks/Lock;->unlock()V

    .line 332
    nop

    .line 333
    return-void

    .line 327
    :cond_18
    :try_start_18
    new-instance v0, Ljava/util/concurrent/RejectedExecutionException;

    const-string v1, "Executor already shutdown"

    invoke-direct {v0, v1}, Ljava/util/concurrent/RejectedExecutionException;-><init>(Ljava/lang/String;)V

    throw v0
    :try_end_20
    .catchall {:try_start_18 .. :try_end_20} :catchall_20

    .line 331
    :catchall_20
    move-exception v0

    iget-object v1, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v1}, Ljava/util/concurrent/locks/Lock;->unlock()V

    throw v0
.end method


# virtual methods
.method public awaitTermination(JLjava/util/concurrent/TimeUnit;)Z
    .registers 9
    .param p1, "timeout"    # J
    .param p3, "unit"    # Ljava/util/concurrent/TimeUnit;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/InterruptedException;
        }
    .end annotation

    .line 299
    invoke-virtual {p3, p1, p2}, Ljava/util/concurrent/TimeUnit;->toNanos(J)J

    move-result-wide v0

    .line 300
    .local v0, "nanos":J
    iget-object v2, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v2}, Ljava/util/concurrent/locks/Lock;->lock()V

    .line 303
    :goto_9
    :try_start_9
    invoke-virtual {p0}, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->isTerminated()Z

    move-result v2
    :try_end_d
    .catchall {:try_start_9 .. :try_end_d} :catchall_26

    if-eqz v2, :cond_16

    .line 304
    const/4 v2, 0x1

    .line 312
    .end local v0    # "nanos":J
    .end local p0    # "this":Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;
    .end local p1    # "timeout":J
    .end local p3    # "unit":Ljava/util/concurrent/TimeUnit;
    :goto_10
    iget-object v3, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v3}, Ljava/util/concurrent/locks/Lock;->unlock()V

    return v2

    .line 305
    .restart local v0    # "nanos":J
    .restart local p0    # "this":Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;
    .restart local p1    # "timeout":J
    .restart local p3    # "unit":Ljava/util/concurrent/TimeUnit;
    :cond_16
    const-wide/16 v2, 0x0

    cmp-long v4, v0, v2

    if-gtz v4, :cond_1e

    .line 306
    const/4 v2, 0x0

    goto :goto_10

    .line 308
    :cond_1e
    :try_start_1e
    iget-object v2, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->termination:Ljava/util/concurrent/locks/Condition;

    invoke-interface {v2, v0, v1}, Ljava/util/concurrent/locks/Condition;->awaitNanos(J)J

    move-result-wide v2
    :try_end_24
    .catchall {:try_start_1e .. :try_end_24} :catchall_26

    move-wide v0, v2

    goto :goto_9

    .line 312
    :catchall_26
    move-exception v2

    iget-object v3, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v3}, Ljava/util/concurrent/locks/Lock;->unlock()V

    throw v2
.end method

.method public execute(Ljava/lang/Runnable;)V
    .registers 3
    .param p1, "command"    # Ljava/lang/Runnable;

    .line 251
    invoke-direct {p0}, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->startTask()V

    .line 253
    :try_start_3
    invoke-interface {p1}, Ljava/lang/Runnable;->run()V
    :try_end_6
    .catchall {:try_start_3 .. :try_end_6} :catchall_b

    .line 255
    invoke-direct {p0}, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->endTask()V

    .line 256
    nop

    .line 257
    return-void

    .line 255
    :catchall_b
    move-exception v0

    invoke-direct {p0}, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->endTask()V

    throw v0
.end method

.method public isShutdown()Z
    .registers 3

    .line 261
    iget-object v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v0}, Ljava/util/concurrent/locks/Lock;->lock()V

    .line 263
    :try_start_5
    iget-boolean v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->shutdown:Z
    :try_end_7
    .catchall {:try_start_5 .. :try_end_7} :catchall_d

    .line 265
    iget-object v1, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v1}, Ljava/util/concurrent/locks/Lock;->unlock()V

    return v0

    :catchall_d
    move-exception v0

    iget-object v1, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v1}, Ljava/util/concurrent/locks/Lock;->unlock()V

    throw v0
.end method

.method public isTerminated()Z
    .registers 3

    .line 288
    iget-object v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v0}, Ljava/util/concurrent/locks/Lock;->lock()V

    .line 290
    :try_start_5
    iget-boolean v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->shutdown:Z

    if-eqz v0, :cond_f

    iget v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->runningTasks:I
    :try_end_b
    .catchall {:try_start_5 .. :try_end_b} :catchall_16

    if-nez v0, :cond_f

    const/4 v0, 0x1

    goto :goto_10

    :cond_f
    const/4 v0, 0x0

    .line 292
    :goto_10
    iget-object v1, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v1}, Ljava/util/concurrent/locks/Lock;->unlock()V

    return v0

    :catchall_16
    move-exception v0

    iget-object v1, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v1}, Ljava/util/concurrent/locks/Lock;->unlock()V

    throw v0
.end method

.method public shutdown()V
    .registers 3

    .line 271
    iget-object v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v0}, Ljava/util/concurrent/locks/Lock;->lock()V

    .line 273
    const/4 v0, 0x1

    :try_start_6
    iput-boolean v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->shutdown:Z
    :try_end_8
    .catchall {:try_start_6 .. :try_end_8} :catchall_f

    .line 275
    iget-object v0, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v0}, Ljava/util/concurrent/locks/Lock;->unlock()V

    .line 276
    nop

    .line 277
    return-void

    .line 275
    :catchall_f
    move-exception v0

    iget-object v1, p0, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->lock:Ljava/util/concurrent/locks/Lock;

    invoke-interface {v1}, Ljava/util/concurrent/locks/Lock;->unlock()V

    throw v0
.end method

.method public shutdownNow()Ljava/util/List;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/List<",
            "Ljava/lang/Runnable;",
            ">;"
        }
    .end annotation

    .line 282
    invoke-virtual {p0}, Lcom/google/common/util/concurrent/MoreExecutors$SameThreadExecutorService;->shutdown()V

    .line 283
    invoke-static {}, Ljava/util/Collections;->emptyList()Ljava/util/List;

    move-result-object v0

    return-object v0
.end method
