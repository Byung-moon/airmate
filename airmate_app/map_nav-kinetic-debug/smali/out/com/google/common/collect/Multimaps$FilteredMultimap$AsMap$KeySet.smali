.class Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet;
.super Lcom/google/common/collect/Maps$KeySet;
.source "Multimaps.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x0
    name = "KeySet"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Lcom/google/common/collect/Maps$KeySet<",
        "TK;",
        "Ljava/util/Collection<",
        "TV;>;>;"
    }
.end annotation


# instance fields
.field final synthetic this$1:Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap;


# direct methods
.method constructor <init>(Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap;)V
    .registers 2

    .line 2522
    .local p0, "this":Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet;, "Lcom/google/common/collect/Multimaps$FilteredMultimap<TK;TV;>.AsMap.KeySet;"
    iput-object p1, p0, Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet;->this$1:Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap;

    invoke-direct {p0}, Lcom/google/common/collect/Maps$KeySet;-><init>()V

    return-void
.end method


# virtual methods
.method map()Ljava/util/Map;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/Map<",
            "TK;",
            "Ljava/util/Collection<",
            "TV;>;>;"
        }
    .end annotation

    .line 2524
    .local p0, "this":Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet;, "Lcom/google/common/collect/Multimaps$FilteredMultimap<TK;TV;>.AsMap.KeySet;"
    iget-object v0, p0, Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet;->this$1:Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap;

    return-object v0
.end method

.method public remove(Ljava/lang/Object;)Z
    .registers 4
    .param p1, "o"    # Ljava/lang/Object;

    .line 2528
    .local p0, "this":Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet;, "Lcom/google/common/collect/Multimaps$FilteredMultimap<TK;TV;>.AsMap.KeySet;"
    iget-object v0, p0, Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet;->this$1:Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap;

    iget-object v0, v0, Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap;->delegate:Ljava/util/Map;

    invoke-interface {v0, p1}, Ljava/util/Map;->get(Ljava/lang/Object;)Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/util/Collection;

    .line 2529
    .local v0, "collection":Ljava/util/Collection;, "Ljava/util/Collection<TV;>;"
    if-nez v0, :cond_e

    .line 2530
    const/4 v1, 0x0

    return v1

    .line 2532
    :cond_e
    invoke-interface {v0}, Ljava/util/Collection;->clear()V

    .line 2533
    const/4 v1, 0x1

    return v1
.end method

.method public removeAll(Ljava/util/Collection;)Z
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/util/Collection<",
            "*>;)Z"
        }
    .end annotation

    .line 2537
    .local p0, "this":Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet;, "Lcom/google/common/collect/Multimaps$FilteredMultimap<TK;TV;>.AsMap.KeySet;"
    .local p1, "c":Ljava/util/Collection;, "Ljava/util/Collection<*>;"
    invoke-interface {p1}, Ljava/util/Collection;->iterator()Ljava/util/Iterator;

    move-result-object v0

    invoke-static {p0, v0}, Lcom/google/common/collect/Sets;->removeAllImpl(Ljava/util/Set;Ljava/util/Iterator;)Z

    move-result v0

    return v0
.end method

.method public retainAll(Ljava/util/Collection;)Z
    .registers 4
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/util/Collection<",
            "*>;)Z"
        }
    .end annotation

    .line 2541
    .local p0, "this":Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet;, "Lcom/google/common/collect/Multimaps$FilteredMultimap<TK;TV;>.AsMap.KeySet;"
    .local p1, "c":Ljava/util/Collection;, "Ljava/util/Collection<*>;"
    new-instance v0, Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet$1;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet$1;-><init>(Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet;Ljava/util/Collection;)V

    .line 2547
    .local v0, "removalPredicate":Lcom/google/common/base/Predicate;, "Lcom/google/common/base/Predicate<Ljava/util/Map$Entry<TK;Ljava/util/Collection<TV;>;>;>;"
    iget-object v1, p0, Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap$KeySet;->this$1:Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap;

    iget-object v1, v1, Lcom/google/common/collect/Multimaps$FilteredMultimap$AsMap;->this$0:Lcom/google/common/collect/Multimaps$FilteredMultimap;

    invoke-virtual {v1, v0}, Lcom/google/common/collect/Multimaps$FilteredMultimap;->removeEntriesIf(Lcom/google/common/base/Predicate;)Z

    move-result v1

    return v1
.end method
