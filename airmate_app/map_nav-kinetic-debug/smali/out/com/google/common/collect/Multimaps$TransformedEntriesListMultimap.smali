.class final Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;
.super Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap;
.source "Multimaps.java"

# interfaces
.implements Lcom/google/common/collect/ListMultimap;


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/Multimaps;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x1a
    name = "TransformedEntriesListMultimap"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<K:",
        "Ljava/lang/Object;",
        "V1:",
        "Ljava/lang/Object;",
        "V2:",
        "Ljava/lang/Object;",
        ">",
        "Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap<",
        "TK;TV1;TV2;>;",
        "Lcom/google/common/collect/ListMultimap<",
        "TK;TV2;>;"
    }
.end annotation


# direct methods
.method constructor <init>(Lcom/google/common/collect/ListMultimap;Lcom/google/common/collect/Maps$EntryTransformer;)V
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Lcom/google/common/collect/ListMultimap<",
            "TK;TV1;>;",
            "Lcom/google/common/collect/Maps$EntryTransformer<",
            "-TK;-TV1;TV2;>;)V"
        }
    .end annotation

    .line 1702
    .local p0, "this":Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;, "Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap<TK;TV1;TV2;>;"
    .local p1, "fromMultimap":Lcom/google/common/collect/ListMultimap;, "Lcom/google/common/collect/ListMultimap<TK;TV1;>;"
    .local p2, "transformer":Lcom/google/common/collect/Maps$EntryTransformer;, "Lcom/google/common/collect/Maps$EntryTransformer<-TK;-TV1;TV2;>;"
    invoke-direct {p0, p1, p2}, Lcom/google/common/collect/Multimaps$TransformedEntriesMultimap;-><init>(Lcom/google/common/collect/Multimap;Lcom/google/common/collect/Maps$EntryTransformer;)V

    .line 1703
    return-void
.end method


# virtual methods
.method public bridge synthetic get(Ljava/lang/Object;)Ljava/util/Collection;
    .registers 3
    .param p1, "x0"    # Ljava/lang/Object;

    .line 1696
    .local p0, "this":Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;, "Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap<TK;TV1;TV2;>;"
    invoke-virtual {p0, p1}, Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;->get(Ljava/lang/Object;)Ljava/util/List;

    move-result-object v0

    return-object v0
.end method

.method public get(Ljava/lang/Object;)Ljava/util/List;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TK;)",
            "Ljava/util/List<",
            "TV2;>;"
        }
    .end annotation

    .line 1714
    .local p0, "this":Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;, "Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap<TK;TV1;TV2;>;"
    .local p1, "key":Ljava/lang/Object;, "TK;"
    iget-object v0, p0, Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;->fromMultimap:Lcom/google/common/collect/Multimap;

    invoke-interface {v0, p1}, Lcom/google/common/collect/Multimap;->get(Ljava/lang/Object;)Ljava/util/Collection;

    move-result-object v0

    invoke-virtual {p0, p1, v0}, Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;->transform(Ljava/lang/Object;Ljava/util/Collection;)Ljava/util/List;

    move-result-object v0

    return-object v0
.end method

.method public bridge synthetic removeAll(Ljava/lang/Object;)Ljava/util/Collection;
    .registers 3
    .param p1, "x0"    # Ljava/lang/Object;

    .line 1696
    .local p0, "this":Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;, "Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap<TK;TV1;TV2;>;"
    invoke-virtual {p0, p1}, Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;->removeAll(Ljava/lang/Object;)Ljava/util/List;

    move-result-object v0

    return-object v0
.end method

.method public removeAll(Ljava/lang/Object;)Ljava/util/List;
    .registers 3
    .param p1, "key"    # Ljava/lang/Object;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/lang/Object;",
            ")",
            "Ljava/util/List<",
            "TV2;>;"
        }
    .end annotation

    .line 1719
    .local p0, "this":Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;, "Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap<TK;TV1;TV2;>;"
    iget-object v0, p0, Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;->fromMultimap:Lcom/google/common/collect/Multimap;

    invoke-interface {v0, p1}, Lcom/google/common/collect/Multimap;->removeAll(Ljava/lang/Object;)Ljava/util/Collection;

    move-result-object v0

    invoke-virtual {p0, p1, v0}, Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;->transform(Ljava/lang/Object;Ljava/util/Collection;)Ljava/util/List;

    move-result-object v0

    return-object v0
.end method

.method public bridge synthetic replaceValues(Ljava/lang/Object;Ljava/lang/Iterable;)Ljava/util/Collection;
    .registers 4
    .param p1, "x0"    # Ljava/lang/Object;
    .param p2, "x1"    # Ljava/lang/Iterable;

    .line 1696
    .local p0, "this":Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;, "Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap<TK;TV1;TV2;>;"
    invoke-virtual {p0, p1, p2}, Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;->replaceValues(Ljava/lang/Object;Ljava/lang/Iterable;)Ljava/util/List;

    move-result-object v0

    return-object v0
.end method

.method public replaceValues(Ljava/lang/Object;Ljava/lang/Iterable;)Ljava/util/List;
    .registers 4
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TK;",
            "Ljava/lang/Iterable<",
            "+TV2;>;)",
            "Ljava/util/List<",
            "TV2;>;"
        }
    .end annotation

    .line 1724
    .local p0, "this":Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;, "Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap<TK;TV1;TV2;>;"
    .local p1, "key":Ljava/lang/Object;, "TK;"
    .local p2, "values":Ljava/lang/Iterable;, "Ljava/lang/Iterable<+TV2;>;"
    new-instance v0, Ljava/lang/UnsupportedOperationException;

    invoke-direct {v0}, Ljava/lang/UnsupportedOperationException;-><init>()V

    throw v0
.end method

.method bridge synthetic transform(Ljava/lang/Object;Ljava/util/Collection;)Ljava/util/Collection;
    .registers 4
    .param p1, "x0"    # Ljava/lang/Object;
    .param p2, "x1"    # Ljava/util/Collection;

    .line 1696
    .local p0, "this":Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;, "Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap<TK;TV1;TV2;>;"
    invoke-virtual {p0, p1, p2}, Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;->transform(Ljava/lang/Object;Ljava/util/Collection;)Ljava/util/List;

    move-result-object v0

    return-object v0
.end method

.method transform(Ljava/lang/Object;Ljava/util/Collection;)Ljava/util/List;
    .registers 5
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TK;",
            "Ljava/util/Collection<",
            "TV1;>;)",
            "Ljava/util/List<",
            "TV2;>;"
        }
    .end annotation

    .line 1706
    .local p0, "this":Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;, "Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap<TK;TV1;TV2;>;"
    .local p1, "key":Ljava/lang/Object;, "TK;"
    .local p2, "values":Ljava/util/Collection;, "Ljava/util/Collection<TV1;>;"
    move-object v0, p2

    check-cast v0, Ljava/util/List;

    new-instance v1, Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap$1;

    invoke-direct {v1, p0, p1}, Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap$1;-><init>(Lcom/google/common/collect/Multimaps$TransformedEntriesListMultimap;Ljava/lang/Object;)V

    invoke-static {v0, v1}, Lcom/google/common/collect/Lists;->transform(Ljava/util/List;Lcom/google/common/base/Function;)Ljava/util/List;

    move-result-object v0

    return-object v0
.end method
