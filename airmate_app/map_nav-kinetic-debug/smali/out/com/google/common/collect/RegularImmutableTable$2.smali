.class final Lcom/google/common/collect/RegularImmutableTable$2;
.super Ljava/lang/Object;
.source "RegularImmutableTable.java"

# interfaces
.implements Ljava/util/Comparator;


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Lcom/google/common/collect/RegularImmutableTable;->forCells(Ljava/util/List;Ljava/util/Comparator;Ljava/util/Comparator;)Lcom/google/common/collect/RegularImmutableTable;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x8
    name = null
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/lang/Object;",
        "Ljava/util/Comparator<",
        "Lcom/google/common/collect/Table$Cell<",
        "TR;TC;TV;>;>;"
    }
.end annotation


# instance fields
.field final synthetic val$columnComparator:Ljava/util/Comparator;

.field final synthetic val$rowComparator:Ljava/util/Comparator;


# direct methods
.method constructor <init>(Ljava/util/Comparator;Ljava/util/Comparator;)V
    .registers 3

    .line 103
    iput-object p1, p0, Lcom/google/common/collect/RegularImmutableTable$2;->val$rowComparator:Ljava/util/Comparator;

    iput-object p2, p0, Lcom/google/common/collect/RegularImmutableTable$2;->val$columnComparator:Ljava/util/Comparator;

    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public compare(Lcom/google/common/collect/Table$Cell;Lcom/google/common/collect/Table$Cell;)I
    .registers 7
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Lcom/google/common/collect/Table$Cell<",
            "TR;TC;TV;>;",
            "Lcom/google/common/collect/Table$Cell<",
            "TR;TC;TV;>;)I"
        }
    .end annotation

    .line 105
    .local p1, "cell1":Lcom/google/common/collect/Table$Cell;, "Lcom/google/common/collect/Table$Cell<TR;TC;TV;>;"
    .local p2, "cell2":Lcom/google/common/collect/Table$Cell;, "Lcom/google/common/collect/Table$Cell<TR;TC;TV;>;"
    iget-object v0, p0, Lcom/google/common/collect/RegularImmutableTable$2;->val$rowComparator:Ljava/util/Comparator;

    const/4 v1, 0x0

    if-nez v0, :cond_7

    const/4 v0, 0x0

    goto :goto_15

    :cond_7
    iget-object v0, p0, Lcom/google/common/collect/RegularImmutableTable$2;->val$rowComparator:Ljava/util/Comparator;

    invoke-interface {p1}, Lcom/google/common/collect/Table$Cell;->getRowKey()Ljava/lang/Object;

    move-result-object v2

    invoke-interface {p2}, Lcom/google/common/collect/Table$Cell;->getRowKey()Ljava/lang/Object;

    move-result-object v3

    invoke-interface {v0, v2, v3}, Ljava/util/Comparator;->compare(Ljava/lang/Object;Ljava/lang/Object;)I

    move-result v0

    .line 107
    .local v0, "rowCompare":I
    :goto_15
    if-eqz v0, :cond_18

    .line 108
    return v0

    .line 110
    :cond_18
    iget-object v2, p0, Lcom/google/common/collect/RegularImmutableTable$2;->val$columnComparator:Ljava/util/Comparator;

    if-nez v2, :cond_1d

    goto :goto_2b

    :cond_1d
    iget-object v1, p0, Lcom/google/common/collect/RegularImmutableTable$2;->val$columnComparator:Ljava/util/Comparator;

    invoke-interface {p1}, Lcom/google/common/collect/Table$Cell;->getColumnKey()Ljava/lang/Object;

    move-result-object v2

    invoke-interface {p2}, Lcom/google/common/collect/Table$Cell;->getColumnKey()Ljava/lang/Object;

    move-result-object v3

    invoke-interface {v1, v2, v3}, Ljava/util/Comparator;->compare(Ljava/lang/Object;Ljava/lang/Object;)I

    move-result v1

    :goto_2b
    return v1
.end method

.method public bridge synthetic compare(Ljava/lang/Object;Ljava/lang/Object;)I
    .registers 5
    .param p1, "x0"    # Ljava/lang/Object;
    .param p2, "x1"    # Ljava/lang/Object;

    .line 103
    move-object v0, p1

    check-cast v0, Lcom/google/common/collect/Table$Cell;

    move-object v1, p2

    check-cast v1, Lcom/google/common/collect/Table$Cell;

    invoke-virtual {p0, v0, v1}, Lcom/google/common/collect/RegularImmutableTable$2;->compare(Lcom/google/common/collect/Table$Cell;Lcom/google/common/collect/Table$Cell;)I

    move-result v0

    return v0
.end method
