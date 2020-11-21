.class public Lorg/apache/commons/net/util/ListenerList;
.super Ljava/lang/Object;
.source "ListenerList.java"

# interfaces
.implements Ljava/io/Serializable;
.implements Ljava/lang/Iterable;


# annotations
.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/lang/Object;",
        "Ljava/io/Serializable;",
        "Ljava/lang/Iterable<",
        "Ljava/util/EventListener;",
        ">;"
    }
.end annotation


# instance fields
.field private final __listeners:Ljava/util/concurrent/CopyOnWriteArrayList;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/concurrent/CopyOnWriteArrayList<",
            "Ljava/util/EventListener;",
            ">;"
        }
    .end annotation
.end field


# direct methods
.method public constructor <init>()V
    .registers 2

    .line 34
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 35
    new-instance v0, Ljava/util/concurrent/CopyOnWriteArrayList;

    invoke-direct {v0}, Ljava/util/concurrent/CopyOnWriteArrayList;-><init>()V

    iput-object v0, p0, Lorg/apache/commons/net/util/ListenerList;->__listeners:Ljava/util/concurrent/CopyOnWriteArrayList;

    .line 36
    return-void
.end method


# virtual methods
.method public addListener(Ljava/util/EventListener;)V
    .registers 3
    .param p1, "listener"    # Ljava/util/EventListener;

    .line 40
    iget-object v0, p0, Lorg/apache/commons/net/util/ListenerList;->__listeners:Ljava/util/concurrent/CopyOnWriteArrayList;

    invoke-virtual {v0, p1}, Ljava/util/concurrent/CopyOnWriteArrayList;->add(Ljava/lang/Object;)Z

    .line 41
    return-void
.end method

.method public getListenerCount()I
    .registers 2

    .line 50
    iget-object v0, p0, Lorg/apache/commons/net/util/ListenerList;->__listeners:Ljava/util/concurrent/CopyOnWriteArrayList;

    invoke-virtual {v0}, Ljava/util/concurrent/CopyOnWriteArrayList;->size()I

    move-result v0

    return v0
.end method

.method public iterator()Ljava/util/Iterator;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/Iterator<",
            "Ljava/util/EventListener;",
            ">;"
        }
    .end annotation

    .line 60
    iget-object v0, p0, Lorg/apache/commons/net/util/ListenerList;->__listeners:Ljava/util/concurrent/CopyOnWriteArrayList;

    invoke-virtual {v0}, Ljava/util/concurrent/CopyOnWriteArrayList;->iterator()Ljava/util/Iterator;

    move-result-object v0

    return-object v0
.end method

.method public removeListener(Ljava/util/EventListener;)V
    .registers 3
    .param p1, "listener"    # Ljava/util/EventListener;

    .line 45
    iget-object v0, p0, Lorg/apache/commons/net/util/ListenerList;->__listeners:Ljava/util/concurrent/CopyOnWriteArrayList;

    invoke-virtual {v0, p1}, Ljava/util/concurrent/CopyOnWriteArrayList;->remove(Ljava/lang/Object;)Z

    .line 46
    return-void
.end method
