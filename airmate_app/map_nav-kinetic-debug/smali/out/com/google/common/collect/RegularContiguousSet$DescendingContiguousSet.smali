.class final Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;
.super Lcom/google/common/collect/ImmutableSortedSet;
.source "RegularContiguousSet.java"


# annotations
.annotation build Lcom/google/common/annotations/GwtIncompatible;
    value = "NavigableSet"
.end annotation

.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/RegularContiguousSet;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x12
    name = "DescendingContiguousSet"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Lcom/google/common/collect/ImmutableSortedSet<",
        "TC;>;"
    }
.end annotation


# instance fields
.field final synthetic this$0:Lcom/google/common/collect/RegularContiguousSet;


# direct methods
.method private constructor <init>(Lcom/google/common/collect/RegularContiguousSet;)V
    .registers 2

    .line 207
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    iput-object p1, p0, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->this$0:Lcom/google/common/collect/RegularContiguousSet;

    .line 208
    invoke-static {}, Lcom/google/common/collect/Ordering;->natural()Lcom/google/common/collect/Ordering;

    move-result-object p1

    invoke-virtual {p1}, Lcom/google/common/collect/Ordering;->reverse()Lcom/google/common/collect/Ordering;

    move-result-object p1

    invoke-direct {p0, p1}, Lcom/google/common/collect/ImmutableSortedSet;-><init>(Ljava/util/Comparator;)V

    .line 209
    return-void
.end method

.method synthetic constructor <init>(Lcom/google/common/collect/RegularContiguousSet;Lcom/google/common/collect/RegularContiguousSet$1;)V
    .registers 3
    .param p1, "x0"    # Lcom/google/common/collect/RegularContiguousSet;
    .param p2, "x1"    # Lcom/google/common/collect/RegularContiguousSet$1;

    .line 205
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    invoke-direct {p0, p1}, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;-><init>(Lcom/google/common/collect/RegularContiguousSet;)V

    return-void
.end method


# virtual methods
.method createDescendingSet()Lcom/google/common/collect/ImmutableSortedSet;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Lcom/google/common/collect/ImmutableSortedSet<",
            "TC;>;"
        }
    .end annotation

    .line 263
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    iget-object v0, p0, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->this$0:Lcom/google/common/collect/RegularContiguousSet;

    return-object v0
.end method

.method public first()Ljava/lang/Comparable;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()TC;"
        }
    .end annotation

    .line 213
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    iget-object v0, p0, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->this$0:Lcom/google/common/collect/RegularContiguousSet;

    invoke-virtual {v0}, Lcom/google/common/collect/RegularContiguousSet;->last()Ljava/lang/Comparable;

    move-result-object v0

    return-object v0
.end method

.method public bridge synthetic first()Ljava/lang/Object;
    .registers 2

    .line 204
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    invoke-virtual {p0}, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->first()Ljava/lang/Comparable;

    move-result-object v0

    return-object v0
.end method

.method headSetImpl(Ljava/lang/Comparable;Z)Lcom/google/common/collect/ImmutableSortedSet;
    .registers 4
    .param p2, "inclusive"    # Z
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TC;Z)",
            "Lcom/google/common/collect/ImmutableSortedSet<",
            "TC;>;"
        }
    .end annotation

    .line 240
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    .local p1, "toElement":Ljava/lang/Comparable;, "TC;"
    iget-object v0, p0, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->this$0:Lcom/google/common/collect/RegularContiguousSet;

    invoke-virtual {v0, p1, p2}, Lcom/google/common/collect/RegularContiguousSet;->tailSetImpl(Ljava/lang/Comparable;Z)Lcom/google/common/collect/ContiguousSet;

    move-result-object v0

    invoke-virtual {v0}, Lcom/google/common/collect/ContiguousSet;->descendingSet()Lcom/google/common/collect/ImmutableSortedSet;

    move-result-object v0

    return-object v0
.end method

.method bridge synthetic headSetImpl(Ljava/lang/Object;Z)Lcom/google/common/collect/ImmutableSortedSet;
    .registers 4
    .param p1, "x0"    # Ljava/lang/Object;
    .param p2, "x1"    # Z

    .line 204
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    move-object v0, p1

    check-cast v0, Ljava/lang/Comparable;

    invoke-virtual {p0, v0, p2}, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->headSetImpl(Ljava/lang/Comparable;Z)Lcom/google/common/collect/ImmutableSortedSet;

    move-result-object v0

    return-object v0
.end method

.method indexOf(Ljava/lang/Object;)I
    .registers 5
    .param p1, "target"    # Ljava/lang/Object;

    .line 268
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    invoke-virtual {p0, p1}, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->contains(Ljava/lang/Object;)Z

    move-result v0

    if-eqz v0, :cond_17

    iget-object v0, p0, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->this$0:Lcom/google/common/collect/RegularContiguousSet;

    iget-object v0, v0, Lcom/google/common/collect/RegularContiguousSet;->domain:Lcom/google/common/collect/DiscreteDomain;

    invoke-virtual {p0}, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->last()Ljava/lang/Comparable;

    move-result-object v1

    move-object v2, p1

    check-cast v2, Ljava/lang/Comparable;

    invoke-virtual {v0, v1, v2}, Lcom/google/common/collect/DiscreteDomain;->distance(Ljava/lang/Comparable;Ljava/lang/Comparable;)J

    move-result-wide v0

    long-to-int v0, v0

    goto :goto_18

    :cond_17
    const/4 v0, -0x1

    :goto_18
    return v0
.end method

.method isPartialView()Z
    .registers 2

    .line 273
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    const/4 v0, 0x0

    return v0
.end method

.method public iterator()Lcom/google/common/collect/UnmodifiableIterator;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Lcom/google/common/collect/UnmodifiableIterator<",
            "TC;>;"
        }
    .end annotation

    .line 228
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    new-instance v0, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet$1;

    invoke-virtual {p0}, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->first()Ljava/lang/Comparable;

    move-result-object v1

    invoke-direct {v0, p0, v1}, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet$1;-><init>(Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;Ljava/lang/Comparable;)V

    return-object v0
.end method

.method public bridge synthetic iterator()Ljava/util/Iterator;
    .registers 2

    .line 204
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    invoke-virtual {p0}, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->iterator()Lcom/google/common/collect/UnmodifiableIterator;

    move-result-object v0

    return-object v0
.end method

.method public last()Ljava/lang/Comparable;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()TC;"
        }
    .end annotation

    .line 218
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    iget-object v0, p0, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->this$0:Lcom/google/common/collect/RegularContiguousSet;

    invoke-virtual {v0}, Lcom/google/common/collect/RegularContiguousSet;->first()Ljava/lang/Comparable;

    move-result-object v0

    return-object v0
.end method

.method public bridge synthetic last()Ljava/lang/Object;
    .registers 2

    .line 204
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    invoke-virtual {p0}, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->last()Ljava/lang/Comparable;

    move-result-object v0

    return-object v0
.end method

.method public size()I
    .registers 2

    .line 223
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    iget-object v0, p0, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->this$0:Lcom/google/common/collect/RegularContiguousSet;

    invoke-virtual {v0}, Lcom/google/common/collect/RegularContiguousSet;->size()I

    move-result v0

    return v0
.end method

.method subSetImpl(Ljava/lang/Comparable;ZLjava/lang/Comparable;Z)Lcom/google/common/collect/ImmutableSortedSet;
    .registers 6
    .param p2, "fromInclusive"    # Z
    .param p4, "toInclusive"    # Z
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TC;ZTC;Z)",
            "Lcom/google/common/collect/ImmutableSortedSet<",
            "TC;>;"
        }
    .end annotation

    .line 249
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    .local p1, "fromElement":Ljava/lang/Comparable;, "TC;"
    .local p3, "toElement":Ljava/lang/Comparable;, "TC;"
    iget-object v0, p0, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->this$0:Lcom/google/common/collect/RegularContiguousSet;

    invoke-virtual {v0, p3, p4, p1, p2}, Lcom/google/common/collect/RegularContiguousSet;->subSetImpl(Ljava/lang/Comparable;ZLjava/lang/Comparable;Z)Lcom/google/common/collect/ContiguousSet;

    move-result-object v0

    invoke-virtual {v0}, Lcom/google/common/collect/ContiguousSet;->descendingSet()Lcom/google/common/collect/ImmutableSortedSet;

    move-result-object v0

    return-object v0
.end method

.method bridge synthetic subSetImpl(Ljava/lang/Object;ZLjava/lang/Object;Z)Lcom/google/common/collect/ImmutableSortedSet;
    .registers 7
    .param p1, "x0"    # Ljava/lang/Object;
    .param p2, "x1"    # Z
    .param p3, "x2"    # Ljava/lang/Object;
    .param p4, "x3"    # Z

    .line 204
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    move-object v0, p1

    check-cast v0, Ljava/lang/Comparable;

    move-object v1, p3

    check-cast v1, Ljava/lang/Comparable;

    invoke-virtual {p0, v0, p2, v1, p4}, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->subSetImpl(Ljava/lang/Comparable;ZLjava/lang/Comparable;Z)Lcom/google/common/collect/ImmutableSortedSet;

    move-result-object v0

    return-object v0
.end method

.method tailSetImpl(Ljava/lang/Comparable;Z)Lcom/google/common/collect/ImmutableSortedSet;
    .registers 4
    .param p2, "inclusive"    # Z
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TC;Z)",
            "Lcom/google/common/collect/ImmutableSortedSet<",
            "TC;>;"
        }
    .end annotation

    .line 258
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    .local p1, "fromElement":Ljava/lang/Comparable;, "TC;"
    iget-object v0, p0, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->this$0:Lcom/google/common/collect/RegularContiguousSet;

    invoke-virtual {v0, p1, p2}, Lcom/google/common/collect/RegularContiguousSet;->headSetImpl(Ljava/lang/Comparable;Z)Lcom/google/common/collect/ContiguousSet;

    move-result-object v0

    invoke-virtual {v0}, Lcom/google/common/collect/ContiguousSet;->descendingSet()Lcom/google/common/collect/ImmutableSortedSet;

    move-result-object v0

    return-object v0
.end method

.method bridge synthetic tailSetImpl(Ljava/lang/Object;Z)Lcom/google/common/collect/ImmutableSortedSet;
    .registers 4
    .param p1, "x0"    # Ljava/lang/Object;
    .param p2, "x1"    # Z

    .line 204
    .local p0, "this":Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;, "Lcom/google/common/collect/RegularContiguousSet<TC;>.DescendingContiguousSet;"
    move-object v0, p1

    check-cast v0, Ljava/lang/Comparable;

    invoke-virtual {p0, v0, p2}, Lcom/google/common/collect/RegularContiguousSet$DescendingContiguousSet;->tailSetImpl(Ljava/lang/Comparable;Z)Lcom/google/common/collect/ImmutableSortedSet;

    move-result-object v0

    return-object v0
.end method
