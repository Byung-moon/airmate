.class final Lcom/google/common/collect/Collections2$OrderedPermutationIterator;
.super Lcom/google/common/collect/AbstractIterator;
.source "Collections2.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/Collections2;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x1a
    name = "OrderedPermutationIterator"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<E:",
        "Ljava/lang/Object;",
        ">",
        "Lcom/google/common/collect/AbstractIterator<",
        "Ljava/util/List<",
        "TE;>;>;"
    }
.end annotation


# instance fields
.field final comparator:Ljava/util/Comparator;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/Comparator<",
            "-TE;>;"
        }
    .end annotation
.end field

.field nextPermutation:Ljava/util/List;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/List<",
            "TE;>;"
        }
    .end annotation
.end field


# direct methods
.method constructor <init>(Ljava/util/List;Ljava/util/Comparator;)V
    .registers 4
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/util/List<",
            "TE;>;",
            "Ljava/util/Comparator<",
            "-TE;>;)V"
        }
    .end annotation

    .line 516
    .local p0, "this":Lcom/google/common/collect/Collections2$OrderedPermutationIterator;, "Lcom/google/common/collect/Collections2$OrderedPermutationIterator<TE;>;"
    .local p1, "list":Ljava/util/List;, "Ljava/util/List<TE;>;"
    .local p2, "comparator":Ljava/util/Comparator;, "Ljava/util/Comparator<-TE;>;"
    invoke-direct {p0}, Lcom/google/common/collect/AbstractIterator;-><init>()V

    .line 517
    invoke-static {p1}, Lcom/google/common/collect/Lists;->newArrayList(Ljava/lang/Iterable;)Ljava/util/ArrayList;

    move-result-object v0

    iput-object v0, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    .line 518
    iput-object p2, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->comparator:Ljava/util/Comparator;

    .line 519
    return-void
.end method


# virtual methods
.method calculateNextPermutation()V
    .registers 6

    .line 531
    .local p0, "this":Lcom/google/common/collect/Collections2$OrderedPermutationIterator;, "Lcom/google/common/collect/Collections2$OrderedPermutationIterator<TE;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->findNextJ()I

    move-result v0

    .line 532
    .local v0, "j":I
    const/4 v1, -0x1

    if-ne v0, v1, :cond_b

    .line 533
    const/4 v1, 0x0

    iput-object v1, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    .line 534
    return-void

    .line 537
    :cond_b
    invoke-virtual {p0, v0}, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->findNextL(I)I

    move-result v1

    .line 538
    .local v1, "l":I
    iget-object v2, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    invoke-static {v2, v0, v1}, Ljava/util/Collections;->swap(Ljava/util/List;II)V

    .line 539
    iget-object v2, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    invoke-interface {v2}, Ljava/util/List;->size()I

    move-result v2

    .line 540
    .local v2, "n":I
    iget-object v3, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    add-int/lit8 v4, v0, 0x1

    invoke-interface {v3, v4, v2}, Ljava/util/List;->subList(II)Ljava/util/List;

    move-result-object v3

    invoke-static {v3}, Ljava/util/Collections;->reverse(Ljava/util/List;)V

    .line 541
    return-void
.end method

.method protected bridge synthetic computeNext()Ljava/lang/Object;
    .registers 2

    .line 509
    .local p0, "this":Lcom/google/common/collect/Collections2$OrderedPermutationIterator;, "Lcom/google/common/collect/Collections2$OrderedPermutationIterator<TE;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->computeNext()Ljava/util/List;

    move-result-object v0

    return-object v0
.end method

.method protected computeNext()Ljava/util/List;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/List<",
            "TE;>;"
        }
    .end annotation

    .line 522
    .local p0, "this":Lcom/google/common/collect/Collections2$OrderedPermutationIterator;, "Lcom/google/common/collect/Collections2$OrderedPermutationIterator<TE;>;"
    iget-object v0, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    if-nez v0, :cond_b

    .line 523
    invoke-virtual {p0}, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->endOfData()Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/util/List;

    return-object v0

    .line 525
    :cond_b
    iget-object v0, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    invoke-static {v0}, Lcom/google/common/collect/ImmutableList;->copyOf(Ljava/util/Collection;)Lcom/google/common/collect/ImmutableList;

    move-result-object v0

    .line 526
    .local v0, "next":Lcom/google/common/collect/ImmutableList;, "Lcom/google/common/collect/ImmutableList<TE;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->calculateNextPermutation()V

    .line 527
    return-object v0
.end method

.method findNextJ()I
    .registers 6

    .line 544
    .local p0, "this":Lcom/google/common/collect/Collections2$OrderedPermutationIterator;, "Lcom/google/common/collect/Collections2$OrderedPermutationIterator<TE;>;"
    iget-object v0, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    invoke-interface {v0}, Ljava/util/List;->size()I

    move-result v0

    add-int/lit8 v0, v0, -0x2

    .local v0, "k":I
    :goto_8
    if-ltz v0, :cond_24

    .line 545
    iget-object v1, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->comparator:Ljava/util/Comparator;

    iget-object v2, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    invoke-interface {v2, v0}, Ljava/util/List;->get(I)Ljava/lang/Object;

    move-result-object v2

    iget-object v3, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    add-int/lit8 v4, v0, 0x1

    invoke-interface {v3, v4}, Ljava/util/List;->get(I)Ljava/lang/Object;

    move-result-object v3

    invoke-interface {v1, v2, v3}, Ljava/util/Comparator;->compare(Ljava/lang/Object;Ljava/lang/Object;)I

    move-result v1

    if-gez v1, :cond_21

    .line 547
    return v0

    .line 544
    :cond_21
    add-int/lit8 v0, v0, -0x1

    goto :goto_8

    .line 550
    .end local v0    # "k":I
    :cond_24
    const/4 v0, -0x1

    return v0
.end method

.method findNextL(I)I
    .registers 6
    .param p1, "j"    # I

    .line 554
    .local p0, "this":Lcom/google/common/collect/Collections2$OrderedPermutationIterator;, "Lcom/google/common/collect/Collections2$OrderedPermutationIterator<TE;>;"
    iget-object v0, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    invoke-interface {v0, p1}, Ljava/util/List;->get(I)Ljava/lang/Object;

    move-result-object v0

    .line 555
    .local v0, "ak":Ljava/lang/Object;, "TE;"
    iget-object v1, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    invoke-interface {v1}, Ljava/util/List;->size()I

    move-result v1

    add-int/lit8 v1, v1, -0x1

    .local v1, "l":I
    :goto_e
    if-le v1, p1, :cond_22

    .line 556
    iget-object v2, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->comparator:Ljava/util/Comparator;

    iget-object v3, p0, Lcom/google/common/collect/Collections2$OrderedPermutationIterator;->nextPermutation:Ljava/util/List;

    invoke-interface {v3, v1}, Ljava/util/List;->get(I)Ljava/lang/Object;

    move-result-object v3

    invoke-interface {v2, v0, v3}, Ljava/util/Comparator;->compare(Ljava/lang/Object;Ljava/lang/Object;)I

    move-result v2

    if-gez v2, :cond_1f

    .line 557
    return v1

    .line 555
    :cond_1f
    add-int/lit8 v1, v1, -0x1

    goto :goto_e

    .line 560
    .end local v1    # "l":I
    :cond_22
    new-instance v1, Ljava/lang/AssertionError;

    const-string v2, "this statement should be unreachable"

    invoke-direct {v1, v2}, Ljava/lang/AssertionError;-><init>(Ljava/lang/Object;)V

    throw v1
.end method
