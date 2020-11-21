.class Lcom/google/common/collect/StandardRowSortedTable;
.super Lcom/google/common/collect/StandardTable;
.source "StandardRowSortedTable.java"

# interfaces
.implements Lcom/google/common/collect/RowSortedTable;


# annotations
.annotation build Lcom/google/common/annotations/GwtCompatible;
.end annotation

.annotation system Ldalvik/annotation/MemberClasses;
    value = {
        Lcom/google/common/collect/StandardRowSortedTable$RowSortedMap;,
        Lcom/google/common/collect/StandardRowSortedTable$RowKeySortedSet;
    }
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<R:",
        "Ljava/lang/Object;",
        "C:",
        "Ljava/lang/Object;",
        "V:",
        "Ljava/lang/Object;",
        ">",
        "Lcom/google/common/collect/StandardTable<",
        "TR;TC;TV;>;",
        "Lcom/google/common/collect/RowSortedTable<",
        "TR;TC;TV;>;"
    }
.end annotation


# static fields
.field private static final serialVersionUID:J


# instance fields
.field private transient rowKeySet:Ljava/util/SortedSet;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/SortedSet<",
            "TR;>;"
        }
    .end annotation
.end field

.field private transient rowMap:Lcom/google/common/collect/StandardRowSortedTable$RowSortedMap;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Lcom/google/common/collect/StandardRowSortedTable<",
            "TR;TC;TV;>.RowSortedMap;"
        }
    .end annotation
.end field


# direct methods
.method constructor <init>(Ljava/util/SortedMap;Lcom/google/common/base/Supplier;)V
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/util/SortedMap<",
            "TR;",
            "Ljava/util/Map<",
            "TC;TV;>;>;",
            "Lcom/google/common/base/Supplier<",
            "+",
            "Ljava/util/Map<",
            "TC;TV;>;>;)V"
        }
    .end annotation

    .line 59
    .local p0, "this":Lcom/google/common/collect/StandardRowSortedTable;, "Lcom/google/common/collect/StandardRowSortedTable<TR;TC;TV;>;"
    .local p1, "backingMap":Ljava/util/SortedMap;, "Ljava/util/SortedMap<TR;Ljava/util/Map<TC;TV;>;>;"
    .local p2, "factory":Lcom/google/common/base/Supplier;, "Lcom/google/common/base/Supplier<+Ljava/util/Map<TC;TV;>;>;"
    invoke-direct {p0, p1, p2}, Lcom/google/common/collect/StandardTable;-><init>(Ljava/util/Map;Lcom/google/common/base/Supplier;)V

    .line 60
    return-void
.end method

.method static synthetic access$100(Lcom/google/common/collect/StandardRowSortedTable;)Ljava/util/SortedMap;
    .registers 2
    .param p0, "x0"    # Lcom/google/common/collect/StandardRowSortedTable;

    .line 49
    invoke-direct {p0}, Lcom/google/common/collect/StandardRowSortedTable;->sortedBackingMap()Ljava/util/SortedMap;

    move-result-object v0

    return-object v0
.end method

.method private sortedBackingMap()Ljava/util/SortedMap;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/SortedMap<",
            "TR;",
            "Ljava/util/Map<",
            "TC;TV;>;>;"
        }
    .end annotation

    .line 63
    .local p0, "this":Lcom/google/common/collect/StandardRowSortedTable;, "Lcom/google/common/collect/StandardRowSortedTable<TR;TC;TV;>;"
    iget-object v0, p0, Lcom/google/common/collect/StandardRowSortedTable;->backingMap:Ljava/util/Map;

    check-cast v0, Ljava/util/SortedMap;

    return-object v0
.end method


# virtual methods
.method public bridge synthetic rowKeySet()Ljava/util/Set;
    .registers 2

    .line 48
    .local p0, "this":Lcom/google/common/collect/StandardRowSortedTable;, "Lcom/google/common/collect/StandardRowSortedTable<TR;TC;TV;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/StandardRowSortedTable;->rowKeySet()Ljava/util/SortedSet;

    move-result-object v0

    return-object v0
.end method

.method public rowKeySet()Ljava/util/SortedSet;
    .registers 4
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/SortedSet<",
            "TR;>;"
        }
    .end annotation

    .line 75
    .local p0, "this":Lcom/google/common/collect/StandardRowSortedTable;, "Lcom/google/common/collect/StandardRowSortedTable<TR;TC;TV;>;"
    iget-object v0, p0, Lcom/google/common/collect/StandardRowSortedTable;->rowKeySet:Ljava/util/SortedSet;

    .line 76
    .local v0, "result":Ljava/util/SortedSet;, "Ljava/util/SortedSet<TR;>;"
    if-nez v0, :cond_d

    new-instance v1, Lcom/google/common/collect/StandardRowSortedTable$RowKeySortedSet;

    const/4 v2, 0x0

    invoke-direct {v1, p0, v2}, Lcom/google/common/collect/StandardRowSortedTable$RowKeySortedSet;-><init>(Lcom/google/common/collect/StandardRowSortedTable;Lcom/google/common/collect/StandardRowSortedTable$1;)V

    iput-object v1, p0, Lcom/google/common/collect/StandardRowSortedTable;->rowKeySet:Ljava/util/SortedSet;

    goto :goto_e

    :cond_d
    move-object v1, v0

    :goto_e
    return-object v1
.end method

.method public bridge synthetic rowMap()Ljava/util/Map;
    .registers 2

    .line 48
    .local p0, "this":Lcom/google/common/collect/StandardRowSortedTable;, "Lcom/google/common/collect/StandardRowSortedTable<TR;TC;TV;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/StandardRowSortedTable;->rowMap()Ljava/util/SortedMap;

    move-result-object v0

    return-object v0
.end method

.method public rowMap()Ljava/util/SortedMap;
    .registers 4
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/SortedMap<",
            "TR;",
            "Ljava/util/Map<",
            "TC;TV;>;>;"
        }
    .end annotation

    .line 128
    .local p0, "this":Lcom/google/common/collect/StandardRowSortedTable;, "Lcom/google/common/collect/StandardRowSortedTable<TR;TC;TV;>;"
    iget-object v0, p0, Lcom/google/common/collect/StandardRowSortedTable;->rowMap:Lcom/google/common/collect/StandardRowSortedTable$RowSortedMap;

    .line 129
    .local v0, "result":Lcom/google/common/collect/StandardRowSortedTable$RowSortedMap;, "Lcom/google/common/collect/StandardRowSortedTable<TR;TC;TV;>.RowSortedMap;"
    if-nez v0, :cond_d

    new-instance v1, Lcom/google/common/collect/StandardRowSortedTable$RowSortedMap;

    const/4 v2, 0x0

    invoke-direct {v1, p0, v2}, Lcom/google/common/collect/StandardRowSortedTable$RowSortedMap;-><init>(Lcom/google/common/collect/StandardRowSortedTable;Lcom/google/common/collect/StandardRowSortedTable$1;)V

    iput-object v1, p0, Lcom/google/common/collect/StandardRowSortedTable;->rowMap:Lcom/google/common/collect/StandardRowSortedTable$RowSortedMap;

    goto :goto_e

    :cond_d
    move-object v1, v0

    :goto_e
    return-object v1
.end method
