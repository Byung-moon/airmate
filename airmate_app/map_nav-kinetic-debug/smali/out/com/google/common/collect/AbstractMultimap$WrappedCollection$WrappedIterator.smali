.class Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;
.super Ljava/lang/Object;
.source "AbstractMultimap.java"

# interfaces
.implements Ljava/util/Iterator;


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/AbstractMultimap$WrappedCollection;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x0
    name = "WrappedIterator"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/lang/Object;",
        "Ljava/util/Iterator<",
        "TV;>;"
    }
.end annotation


# instance fields
.field final delegateIterator:Ljava/util/Iterator;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/Iterator<",
            "TV;>;"
        }
    .end annotation
.end field

.field final originalDelegate:Ljava/util/Collection;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/Collection<",
            "TV;>;"
        }
    .end annotation
.end field

.field final synthetic this$1:Lcom/google/common/collect/AbstractMultimap$WrappedCollection;


# direct methods
.method constructor <init>(Lcom/google/common/collect/AbstractMultimap$WrappedCollection;)V
    .registers 3

    .line 512
    .local p0, "this":Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;, "Lcom/google/common/collect/AbstractMultimap<TK;TV;>.WrappedCollection.WrappedIterator;"
    iput-object p1, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->this$1:Lcom/google/common/collect/AbstractMultimap$WrappedCollection;

    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 510
    iget-object v0, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->this$1:Lcom/google/common/collect/AbstractMultimap$WrappedCollection;

    iget-object v0, v0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection;->delegate:Ljava/util/Collection;

    iput-object v0, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->originalDelegate:Ljava/util/Collection;

    .line 513
    iget-object v0, p1, Lcom/google/common/collect/AbstractMultimap$WrappedCollection;->this$0:Lcom/google/common/collect/AbstractMultimap;

    iget-object p1, p1, Lcom/google/common/collect/AbstractMultimap$WrappedCollection;->delegate:Ljava/util/Collection;

    invoke-static {v0, p1}, Lcom/google/common/collect/AbstractMultimap;->access$100(Lcom/google/common/collect/AbstractMultimap;Ljava/util/Collection;)Ljava/util/Iterator;

    move-result-object p1

    iput-object p1, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->delegateIterator:Ljava/util/Iterator;

    .line 514
    return-void
.end method

.method constructor <init>(Lcom/google/common/collect/AbstractMultimap$WrappedCollection;Ljava/util/Iterator;)V
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/util/Iterator<",
            "TV;>;)V"
        }
    .end annotation

    .line 516
    .local p0, "this":Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;, "Lcom/google/common/collect/AbstractMultimap<TK;TV;>.WrappedCollection.WrappedIterator;"
    .local p2, "delegateIterator":Ljava/util/Iterator;, "Ljava/util/Iterator<TV;>;"
    iput-object p1, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->this$1:Lcom/google/common/collect/AbstractMultimap$WrappedCollection;

    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 510
    iget-object p1, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->this$1:Lcom/google/common/collect/AbstractMultimap$WrappedCollection;

    iget-object p1, p1, Lcom/google/common/collect/AbstractMultimap$WrappedCollection;->delegate:Ljava/util/Collection;

    iput-object p1, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->originalDelegate:Ljava/util/Collection;

    .line 517
    iput-object p2, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->delegateIterator:Ljava/util/Iterator;

    .line 518
    return-void
.end method


# virtual methods
.method getDelegateIterator()Ljava/util/Iterator;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/Iterator<",
            "TV;>;"
        }
    .end annotation

    .line 551
    .local p0, "this":Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;, "Lcom/google/common/collect/AbstractMultimap<TK;TV;>.WrappedCollection.WrappedIterator;"
    invoke-virtual {p0}, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->validateIterator()V

    .line 552
    iget-object v0, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->delegateIterator:Ljava/util/Iterator;

    return-object v0
.end method

.method public hasNext()Z
    .registers 2

    .line 533
    .local p0, "this":Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;, "Lcom/google/common/collect/AbstractMultimap<TK;TV;>.WrappedCollection.WrappedIterator;"
    invoke-virtual {p0}, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->validateIterator()V

    .line 534
    iget-object v0, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->delegateIterator:Ljava/util/Iterator;

    invoke-interface {v0}, Ljava/util/Iterator;->hasNext()Z

    move-result v0

    return v0
.end method

.method public next()Ljava/lang/Object;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()TV;"
        }
    .end annotation

    .line 539
    .local p0, "this":Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;, "Lcom/google/common/collect/AbstractMultimap<TK;TV;>.WrappedCollection.WrappedIterator;"
    invoke-virtual {p0}, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->validateIterator()V

    .line 540
    iget-object v0, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->delegateIterator:Ljava/util/Iterator;

    invoke-interface {v0}, Ljava/util/Iterator;->next()Ljava/lang/Object;

    move-result-object v0

    return-object v0
.end method

.method public remove()V
    .registers 2

    .line 545
    .local p0, "this":Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;, "Lcom/google/common/collect/AbstractMultimap<TK;TV;>.WrappedCollection.WrappedIterator;"
    iget-object v0, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->delegateIterator:Ljava/util/Iterator;

    invoke-interface {v0}, Ljava/util/Iterator;->remove()V

    .line 546
    iget-object v0, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->this$1:Lcom/google/common/collect/AbstractMultimap$WrappedCollection;

    iget-object v0, v0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection;->this$0:Lcom/google/common/collect/AbstractMultimap;

    invoke-static {v0}, Lcom/google/common/collect/AbstractMultimap;->access$210(Lcom/google/common/collect/AbstractMultimap;)I

    .line 547
    iget-object v0, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->this$1:Lcom/google/common/collect/AbstractMultimap$WrappedCollection;

    invoke-virtual {v0}, Lcom/google/common/collect/AbstractMultimap$WrappedCollection;->removeIfEmpty()V

    .line 548
    return-void
.end method

.method validateIterator()V
    .registers 3

    .line 525
    .local p0, "this":Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;, "Lcom/google/common/collect/AbstractMultimap<TK;TV;>.WrappedCollection.WrappedIterator;"
    iget-object v0, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->this$1:Lcom/google/common/collect/AbstractMultimap$WrappedCollection;

    invoke-virtual {v0}, Lcom/google/common/collect/AbstractMultimap$WrappedCollection;->refreshIfEmpty()V

    .line 526
    iget-object v0, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->this$1:Lcom/google/common/collect/AbstractMultimap$WrappedCollection;

    iget-object v0, v0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection;->delegate:Ljava/util/Collection;

    iget-object v1, p0, Lcom/google/common/collect/AbstractMultimap$WrappedCollection$WrappedIterator;->originalDelegate:Ljava/util/Collection;

    if-ne v0, v1, :cond_e

    .line 529
    return-void

    .line 527
    :cond_e
    new-instance v0, Ljava/util/ConcurrentModificationException;

    invoke-direct {v0}, Ljava/util/ConcurrentModificationException;-><init>()V

    throw v0
.end method
