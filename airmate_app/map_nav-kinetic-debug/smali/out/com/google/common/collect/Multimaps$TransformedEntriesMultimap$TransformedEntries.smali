.class Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap$TransformedEntries;
.super Lcom/google/common/collect/Collections2$TransformedCollection;
.source "Multimaps.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x2
    name = "TransformedEntries"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Lcom/google/common/collect/Collections2$TransformedCollection<",
        "Ljava/util/Map$Entry<",
        "TK;TV1;>;",
        "Ljava/util/Map$Entry<",
        "TK;TV2;>;>;"
    }
.end annotation


# instance fields
.field final synthetic this$0:Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap;


# direct methods
.method constructor <init>(Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap;Lcom/google/common/collect/Maps$EntryTransformer;)V
    .registers 5
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Lcom/google/common/collect/Maps$EntryTransformer<",
            "-TK;-TV1;TV2;>;)V"
        }
    .end annotation

    .line 1461
    .local p0, "this":Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap$TransformedEntries;, "Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap<TK;TV1;TV2;>.TransformedEntries;"
    .local p2, "transformer":Lcom/google/common/collect/Maps$EntryTransformer;, "Lcom/google/common/collect/Maps$EntryTransformer<-TK;-TV1;TV2;>;"
    iput-object p1, p0, Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap$TransformedEntries;->this$0:Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap;

    .line 1462
    iget-object v0, p1, Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap;->fromMultimap:Lcom/google/common/collect/Multimap;

    invoke-interface {v0}, Lcom/google/common/collect/Multimap;->entries()Ljava/util/Collection;

    move-result-object v0

    new-instance v1, Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap$TransformedEntries$1;

    invoke-direct {v1, p1, p2}, Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap$TransformedEntries$1;-><init>(Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap;Lcom/google/common/collect/Maps$EntryTransformer;)V

    invoke-direct {p0, v0, v1}, Lcom/google/common/collect/Collections2$TransformedCollection;-><init>(Ljava/util/Collection;Lcom/google/common/base/Function;)V

    .line 1478
    return-void
.end method


# virtual methods
.method public contains(Ljava/lang/Object;)Z
    .registers 6
    .param p1, "o"    # Ljava/lang/Object;

    .line 1481
    .local p0, "this":Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap$TransformedEntries;, "Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap<TK;TV1;TV2;>.TransformedEntries;"
    instance-of v0, p1, Ljava/util/Map$Entry;

    if-eqz v0, :cond_16

    .line 1482
    move-object v0, p1

    check-cast v0, Ljava/util/Map$Entry;

    .line 1483
    .local v0, "entry":Ljava/util/Map$Entry;, "Ljava/util/Map$Entry<**>;"
    iget-object v1, p0, Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap$TransformedEntries;->this$0:Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap;

    invoke-interface {v0}, Ljava/util/Map$Entry;->getKey()Ljava/lang/Object;

    move-result-object v2

    invoke-interface {v0}, Ljava/util/Map$Entry;->getValue()Ljava/lang/Object;

    move-result-object v3

    invoke-virtual {v1, v2, v3}, Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap;->containsEntry(Ljava/lang/Object;Ljava/lang/Object;)Z

    move-result v1

    return v1

    .line 1485
    .end local v0    # "entry":Ljava/util/Map$Entry;, "Ljava/util/Map$Entry<**>;"
    :cond_16
    const/4 v0, 0x0

    return v0
.end method

.method public remove(Ljava/lang/Object;)Z
    .registers 5
    .param p1, "o"    # Ljava/lang/Object;

    .line 1490
    .local p0, "this":Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap$TransformedEntries;, "Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap<TK;TV1;TV2;>.TransformedEntries;"
    instance-of v0, p1, Ljava/util/Map$Entry;

    if-eqz v0, :cond_1a

    .line 1491
    move-object v0, p1

    check-cast v0, Ljava/util/Map$Entry;

    .line 1492
    .local v0, "entry":Ljava/util/Map$Entry;, "Ljava/util/Map$Entry<**>;"
    iget-object v1, p0, Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap$TransformedEntries;->this$0:Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap;

    invoke-interface {v0}, Ljava/util/Map$Entry;->getKey()Ljava/lang/Object;

    move-result-object v2

    invoke-virtual {v1, v2}, Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap;->get(Ljava/lang/Object;)Ljava/util/Collection;

    move-result-object v1

    .line 1493
    .local v1, "values":Ljava/util/Collection;, "Ljava/util/Collection<TV2;>;"
    invoke-interface {v0}, Ljava/util/Map$Entry;->getValue()Ljava/lang/Object;

    move-result-object v2

    invoke-interface {v1, v2}, Ljava/util/Collection;->remove(Ljava/lang/Object;)Z

    move-result v2

    return v2

    .line 1495
    .end local v0    # "entry":Ljava/util/Map$Entry;, "Ljava/util/Map$Entry<**>;"
    .end local v1    # "values":Ljava/util/Collection;, "Ljava/util/Collection<TV2;>;"
    :cond_1a
    const/4 v0, 0x0

    return v0
.end method
