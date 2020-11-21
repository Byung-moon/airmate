.class public Lcom/google/common/eventbus/AsyncEventBus;
.super Lcom/google/common/eventbus/EventBus;
.source "AsyncEventBus.java"


# annotations
.annotation build Lcom/google/common/annotations/Beta;
.end annotation


# instance fields
.field private final eventsToDispatch:Ljava/util/concurrent/ConcurrentLinkedQueue;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/concurrent/ConcurrentLinkedQueue<",
            "Lcom/google/common/eventbus/EventBus$EventWithHandler;",
            ">;"
        }
    .end annotation
.end field

.field private final executor:Ljava/util/concurrent/Executor;


# direct methods
.method public constructor <init>(Ljava/lang/String;Ljava/util/concurrent/Executor;)V
    .registers 4
    .param p1, "identifier"    # Ljava/lang/String;
    .param p2, "executor"    # Ljava/util/concurrent/Executor;

    .line 49
    invoke-direct {p0, p1}, Lcom/google/common/eventbus/EventBus;-><init>(Ljava/lang/String;)V

    .line 36
    new-instance v0, Ljava/util/concurrent/ConcurrentLinkedQueue;

    invoke-direct {v0}, Ljava/util/concurrent/ConcurrentLinkedQueue;-><init>()V

    iput-object v0, p0, Lcom/google/common/eventbus/AsyncEventBus;->eventsToDispatch:Ljava/util/concurrent/ConcurrentLinkedQueue;

    .line 50
    iput-object p2, p0, Lcom/google/common/eventbus/AsyncEventBus;->executor:Ljava/util/concurrent/Executor;

    .line 51
    return-void
.end method

.method public constructor <init>(Ljava/util/concurrent/Executor;)V
    .registers 3
    .param p1, "executor"    # Ljava/util/concurrent/Executor;

    .line 61
    invoke-direct {p0}, Lcom/google/common/eventbus/EventBus;-><init>()V

    .line 36
    new-instance v0, Ljava/util/concurrent/ConcurrentLinkedQueue;

    invoke-direct {v0}, Ljava/util/concurrent/ConcurrentLinkedQueue;-><init>()V

    iput-object v0, p0, Lcom/google/common/eventbus/AsyncEventBus;->eventsToDispatch:Ljava/util/concurrent/ConcurrentLinkedQueue;

    .line 62
    iput-object p1, p0, Lcom/google/common/eventbus/AsyncEventBus;->executor:Ljava/util/concurrent/Executor;

    .line 63
    return-void
.end method

.method static synthetic access$001(Lcom/google/common/eventbus/AsyncEventBus;Ljava/lang/Object;Lcom/google/common/eventbus/EventHandler;)V
    .registers 3
    .param p0, "x0"    # Lcom/google/common/eventbus/AsyncEventBus;
    .param p1, "x1"    # Ljava/lang/Object;
    .param p2, "x2"    # Lcom/google/common/eventbus/EventHandler;

    .line 32
    invoke-super {p0, p1, p2}, Lcom/google/common/eventbus/EventBus;->dispatch(Ljava/lang/Object;Lcom/google/common/eventbus/EventHandler;)V

    return-void
.end method


# virtual methods
.method protected dispatch(Ljava/lang/Object;Lcom/google/common/eventbus/EventHandler;)V
    .registers 5
    .param p1, "event"    # Ljava/lang/Object;
    .param p2, "handler"    # Lcom/google/common/eventbus/EventHandler;

    .line 91
    iget-object v0, p0, Lcom/google/common/eventbus/AsyncEventBus;->executor:Ljava/util/concurrent/Executor;

    new-instance v1, Lcom/google/common/eventbus/AsyncEventBus$1;

    invoke-direct {v1, p0, p1, p2}, Lcom/google/common/eventbus/AsyncEventBus$1;-><init>(Lcom/google/common/eventbus/AsyncEventBus;Ljava/lang/Object;Lcom/google/common/eventbus/EventHandler;)V

    invoke-interface {v0, v1}, Ljava/util/concurrent/Executor;->execute(Ljava/lang/Runnable;)V

    .line 98
    return-void
.end method

.method protected dispatchQueuedEvents()V
    .registers 4

    .line 77
    :goto_0
    iget-object v0, p0, Lcom/google/common/eventbus/AsyncEventBus;->eventsToDispatch:Ljava/util/concurrent/ConcurrentLinkedQueue;

    invoke-virtual {v0}, Ljava/util/concurrent/ConcurrentLinkedQueue;->poll()Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Lcom/google/common/eventbus/EventBus$EventWithHandler;

    .line 78
    .local v0, "eventWithHandler":Lcom/google/common/eventbus/EventBus$EventWithHandler;
    if-nez v0, :cond_c

    .line 79
    nop

    .line 84
    .end local v0    # "eventWithHandler":Lcom/google/common/eventbus/EventBus$EventWithHandler;
    return-void

    .line 82
    .restart local v0    # "eventWithHandler":Lcom/google/common/eventbus/EventBus$EventWithHandler;
    :cond_c
    iget-object v1, v0, Lcom/google/common/eventbus/EventBus$EventWithHandler;->event:Ljava/lang/Object;

    iget-object v2, v0, Lcom/google/common/eventbus/EventBus$EventWithHandler;->handler:Lcom/google/common/eventbus/EventHandler;

    invoke-virtual {p0, v1, v2}, Lcom/google/common/eventbus/AsyncEventBus;->dispatch(Ljava/lang/Object;Lcom/google/common/eventbus/EventHandler;)V

    .line 83
    .end local v0    # "eventWithHandler":Lcom/google/common/eventbus/EventBus$EventWithHandler;
    goto :goto_0
.end method

.method protected enqueueEvent(Ljava/lang/Object;Lcom/google/common/eventbus/EventHandler;)V
    .registers 5
    .param p1, "event"    # Ljava/lang/Object;
    .param p2, "handler"    # Lcom/google/common/eventbus/EventHandler;

    .line 67
    iget-object v0, p0, Lcom/google/common/eventbus/AsyncEventBus;->eventsToDispatch:Ljava/util/concurrent/ConcurrentLinkedQueue;

    new-instance v1, Lcom/google/common/eventbus/EventBus$EventWithHandler;

    invoke-direct {v1, p1, p2}, Lcom/google/common/eventbus/EventBus$EventWithHandler;-><init>(Ljava/lang/Object;Lcom/google/common/eventbus/EventHandler;)V

    invoke-virtual {v0, v1}, Ljava/util/concurrent/ConcurrentLinkedQueue;->offer(Ljava/lang/Object;)Z

    .line 68
    return-void
.end method
