.class abstract Lcom/google/common/collect/Multisets$ElementSet;
.super Ljava/util/AbstractSet;
.source "Multisets.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/Multisets;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x408
    name = "ElementSet"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<E:",
        "Ljava/lang/Object;",
        ">",
        "Ljava/util/AbstractSet<",
        "TE;>;"
    }
.end annotation


# direct methods
.method constructor <init>()V
    .registers 1

    .line 805
    .local p0, "this":Lcom/google/common/collect/Multisets$ElementSet;, "Lcom/google/common/collect/Multisets$ElementSet<TE;>;"
    invoke-direct {p0}, Ljava/util/AbstractSet;-><init>()V

    return-void
.end method


# virtual methods
.method public clear()V
    .registers 2

    .line 809
    .local p0, "this":Lcom/google/common/collect/Multisets$ElementSet;, "Lcom/google/common/collect/Multisets$ElementSet<TE;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/Multisets$ElementSet;->multiset()Lcom/google/common/collect/Multiset;

    move-result-object v0

    invoke-interface {v0}, Lcom/google/common/collect/Multiset;->clear()V

    .line 810
    return-void
.end method

.method public contains(Ljava/lang/Object;)Z
    .registers 3
    .param p1, "o"    # Ljava/lang/Object;

    .line 813
    .local p0, "this":Lcom/google/common/collect/Multisets$ElementSet;, "Lcom/google/common/collect/Multisets$ElementSet<TE;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/Multisets$ElementSet;->multiset()Lcom/google/common/collect/Multiset;

    move-result-object v0

    invoke-interface {v0, p1}, Lcom/google/common/collect/Multiset;->contains(Ljava/lang/Object;)Z

    move-result v0

    return v0
.end method

.method public containsAll(Ljava/util/Collection;)Z
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/util/Collection<",
            "*>;)Z"
        }
    .end annotation

    .line 817
    .local p0, "this":Lcom/google/common/collect/Multisets$ElementSet;, "Lcom/google/common/collect/Multisets$ElementSet<TE;>;"
    .local p1, "c":Ljava/util/Collection;, "Ljava/util/Collection<*>;"
    invoke-virtual {p0}, Lcom/google/common/collect/Multisets$ElementSet;->multiset()Lcom/google/common/collect/Multiset;

    move-result-object v0

    invoke-interface {v0, p1}, Lcom/google/common/collect/Multiset;->containsAll(Ljava/util/Collection;)Z

    move-result v0

    return v0
.end method

.method public isEmpty()Z
    .registers 2

    .line 821
    .local p0, "this":Lcom/google/common/collect/Multisets$ElementSet;, "Lcom/google/common/collect/Multisets$ElementSet<TE;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/Multisets$ElementSet;->multiset()Lcom/google/common/collect/Multiset;

    move-result-object v0

    invoke-interface {v0}, Lcom/google/common/collect/Multiset;->isEmpty()Z

    move-result v0

    return v0
.end method

.method public iterator()Ljava/util/Iterator;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/Iterator<",
            "TE;>;"
        }
    .end annotation

    .line 825
    .local p0, "this":Lcom/google/common/collect/Multisets$ElementSet;, "Lcom/google/common/collect/Multisets$ElementSet<TE;>;"
    new-instance v0, Lcom/google/common/collect/Multisets$ElementSet$1;

    invoke-virtual {p0}, Lcom/google/common/collect/Multisets$ElementSet;->multiset()Lcom/google/common/collect/Multiset;

    move-result-object v1

    invoke-interface {v1}, Lcom/google/common/collect/Multiset;->entrySet()Ljava/util/Set;

    move-result-object v1

    invoke-interface {v1}, Ljava/util/Set;->iterator()Ljava/util/Iterator;

    move-result-object v1

    invoke-direct {v0, p0, v1}, Lcom/google/common/collect/Multisets$ElementSet$1;-><init>(Lcom/google/common/collect/Multisets$ElementSet;Ljava/util/Iterator;)V

    return-object v0
.end method

.method abstract multiset()Lcom/google/common/collect/Multiset;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Lcom/google/common/collect/Multiset<",
            "TE;>;"
        }
    .end annotation
.end method

.method public remove(Ljava/lang/Object;)Z
    .registers 4
    .param p1, "o"    # Ljava/lang/Object;

    .line 835
    .local p0, "this":Lcom/google/common/collect/Multisets$ElementSet;, "Lcom/google/common/collect/Multisets$ElementSet<TE;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/Multisets$ElementSet;->multiset()Lcom/google/common/collect/Multiset;

    move-result-object v0

    invoke-interface {v0, p1}, Lcom/google/common/collect/Multiset;->count(Ljava/lang/Object;)I

    move-result v0

    .line 836
    .local v0, "count":I
    if-lez v0, :cond_13

    .line 837
    invoke-virtual {p0}, Lcom/google/common/collect/Multisets$ElementSet;->multiset()Lcom/google/common/collect/Multiset;

    move-result-object v1

    invoke-interface {v1, p1, v0}, Lcom/google/common/collect/Multiset;->remove(Ljava/lang/Object;I)I

    .line 838
    const/4 v1, 0x1

    return v1

    .line 840
    :cond_13
    const/4 v1, 0x0

    return v1
.end method

.method public size()I
    .registers 2

    .line 844
    .local p0, "this":Lcom/google/common/collect/Multisets$ElementSet;, "Lcom/google/common/collect/Multisets$ElementSet<TE;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/Multisets$ElementSet;->multiset()Lcom/google/common/collect/Multiset;

    move-result-object v0

    invoke-interface {v0}, Lcom/google/common/collect/Multiset;->entrySet()Ljava/util/Set;

    move-result-object v0

    invoke-interface {v0}, Ljava/util/Set;->size()I

    move-result v0

    return v0
.end method
