.class Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet$1;
.super Ljava/lang/Object;
.source "Multimaps.java"

# interfaces
.implements Lcom/google/common/base/Predicate;


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet;->retainAll(Ljava/util/Collection;)Z
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x0
    name = null
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/lang/Object;",
        "Lcom/google/common/base/Predicate<",
        "Ljava/util/Map$Entry<",
        "TK;",
        "Ljava/util/Collection<",
        "TV;>;>;>;"
    }
.end annotation


# instance fields
.field final synthetic this$2:Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet;

.field final synthetic val$c:Ljava/util/Collection;


# direct methods
.method constructor <init>(Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet;Ljava/util/Collection;)V
    .registers 3

    .line 2686
    .local p0, "this":Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet$1;, "Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet.1;"
    iput-object p1, p0, Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet$1;->this$2:Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet;

    iput-object p2, p0, Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet$1;->val$c:Ljava/util/Collection;

    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public bridge synthetic apply(Ljava/lang/Object;)Z
    .registers 3
    .param p1, "x0"    # Ljava/lang/Object;

    .line 2686
    .local p0, "this":Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet$1;, "Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet.1;"
    move-object v0, p1

    check-cast v0, Ljava/util/Map$Entry;

    invoke-virtual {p0, v0}, Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet$1;->apply(Ljava/util/Map$Entry;)Z

    move-result v0

    return v0
.end method

.method public apply(Ljava/util/Map$Entry;)Z
    .registers 4
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/util/Map$Entry<",
            "TK;",
            "Ljava/util/Collection<",
            "TV;>;>;)Z"
        }
    .end annotation

    .line 2688
    .local p0, "this":Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet$1;, "Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet.1;"
    .local p1, "entry":Ljava/util/Map$Entry;, "Ljava/util/Map$Entry<TK;Ljava/util/Collection<TV;>;>;"
    invoke-interface {p1}, Ljava/util/Map$Entry;->getKey()Ljava/lang/Object;

    move-result-object v0

    invoke-interface {p1}, Ljava/util/Map$Entry;->getValue()Ljava/lang/Object;

    move-result-object v1

    check-cast v1, Ljava/util/Collection;

    invoke-interface {v1}, Ljava/util/Collection;->size()I

    move-result v1

    invoke-static {v0, v1}, Lcom/google/common/collect/Multisets;->immutableEntry(Ljava/lang/Object;I)Lcom/google/common/collect/Multiset$Entry;

    move-result-object v0

    .line 2690
    .local v0, "multisetEntry":Lcom/google/common/collect/Multiset$Entry;, "Lcom/google/common/collect/Multiset$Entry<TK;>;"
    iget-object v1, p0, Lcom/google/common/collect/Multimaps$FilteredMultimap$Keys$EntrySet$1;->val$c:Ljava/util/Collection;

    invoke-interface {v1, v0}, Ljava/util/Collection;->contains(Ljava/lang/Object;)Z

    move-result v1

    xor-int/lit8 v1, v1, 0x1

    return v1
.end method
