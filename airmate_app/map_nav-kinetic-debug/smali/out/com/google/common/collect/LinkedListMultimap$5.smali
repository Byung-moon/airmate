.class Lcom/google/common/collect/LinkedListMultimap$5;
.super Ljava/util/AbstractSequentialList;
.source "LinkedListMultimap.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Lcom/google/common/collect/LinkedListMultimap;->entries()Ljava/util/List;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x0
    name = null
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/util/AbstractSequentialList<",
        "Ljava/util/Map$Entry<",
        "TK;TV;>;>;"
    }
.end annotation


# instance fields
.field final synthetic this$0:Lcom/google/common/collect/LinkedListMultimap;


# direct methods
.method constructor <init>(Lcom/google/common/collect/LinkedListMultimap;)V
    .registers 2

    .line 869
    .local p0, "this":Lcom/google/common/collect/LinkedListMultimap$5;, "Lcom/google/common/collect/LinkedListMultimap.5;"
    iput-object p1, p0, Lcom/google/common/collect/LinkedListMultimap$5;->this$0:Lcom/google/common/collect/LinkedListMultimap;

    invoke-direct {p0}, Ljava/util/AbstractSequentialList;-><init>()V

    return-void
.end method


# virtual methods
.method public listIterator(I)Ljava/util/ListIterator;
    .registers 5
    .param p1, "index"    # I
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(I)",
            "Ljava/util/ListIterator<",
            "Ljava/util/Map$Entry<",
            "TK;TV;>;>;"
        }
    .end annotation

    .line 875
    .local p0, "this":Lcom/google/common/collect/LinkedListMultimap$5;, "Lcom/google/common/collect/LinkedListMultimap.5;"
    new-instance v0, Lcom/google/common/collect/LinkedListMultimap$5$1;

    new-instance v1, Lcom/google/common/collect/LinkedListMultimap$NodeIterator;

    iget-object v2, p0, Lcom/google/common/collect/LinkedListMultimap$5;->this$0:Lcom/google/common/collect/LinkedListMultimap;

    invoke-direct {v1, v2, p1}, Lcom/google/common/collect/LinkedListMultimap$NodeIterator;-><init>(Lcom/google/common/collect/LinkedListMultimap;I)V

    invoke-direct {v0, p0, v1}, Lcom/google/common/collect/LinkedListMultimap$5$1;-><init>(Lcom/google/common/collect/LinkedListMultimap$5;Ljava/util/ListIterator;)V

    return-object v0
.end method

.method public size()I
    .registers 2

    .line 871
    .local p0, "this":Lcom/google/common/collect/LinkedListMultimap$5;, "Lcom/google/common/collect/LinkedListMultimap.5;"
    iget-object v0, p0, Lcom/google/common/collect/LinkedListMultimap$5;->this$0:Lcom/google/common/collect/LinkedListMultimap;

    invoke-static {v0}, Lcom/google/common/collect/LinkedListMultimap;->access$600(Lcom/google/common/collect/LinkedListMultimap;)Lcom/google/common/collect/Multiset;

    move-result-object v0

    invoke-interface {v0}, Lcom/google/common/collect/Multiset;->size()I

    move-result v0

    return v0
.end method
