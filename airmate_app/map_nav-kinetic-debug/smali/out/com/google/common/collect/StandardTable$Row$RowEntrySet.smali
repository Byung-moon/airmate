.class Lcom/google/common/collect/StandardTable$Row$RowEntrySet;
.super Lcom/google/common/collect/Maps$EntrySet;
.source "StandardTable.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/StandardTable$Row;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x2
    name = "RowEntrySet"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Lcom/google/common/collect/Maps$EntrySet<",
        "TC;TV;>;"
    }
.end annotation


# instance fields
.field final synthetic this$1:Lcom/google/common/collect/StandardTable$Row;


# direct methods
.method private constructor <init>(Lcom/google/common/collect/StandardTable$Row;)V
    .registers 2

    .line 443
    .local p0, "this":Lcom/google/common/collect/StandardTable$Row$RowEntrySet;, "Lcom/google/common/collect/StandardTable<TR;TC;TV;>.Row.RowEntrySet;"
    iput-object p1, p0, Lcom/google/common/collect/StandardTable$Row$RowEntrySet;->this$1:Lcom/google/common/collect/StandardTable$Row;

    invoke-direct {p0}, Lcom/google/common/collect/Maps$EntrySet;-><init>()V

    return-void
.end method

.method synthetic constructor <init>(Lcom/google/common/collect/StandardTable$Row;Lcom/google/common/collect/StandardTable$1;)V
    .registers 3
    .param p1, "x0"    # Lcom/google/common/collect/StandardTable$Row;
    .param p2, "x1"    # Lcom/google/common/collect/StandardTable$1;

    .line 443
    .local p0, "this":Lcom/google/common/collect/StandardTable$Row$RowEntrySet;, "Lcom/google/common/collect/StandardTable<TR;TC;TV;>.Row.RowEntrySet;"
    invoke-direct {p0, p1}, Lcom/google/common/collect/StandardTable$Row$RowEntrySet;-><init>(Lcom/google/common/collect/StandardTable$Row;)V

    return-void
.end method


# virtual methods
.method public iterator()Ljava/util/Iterator;
    .registers 4
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/Iterator<",
            "Ljava/util/Map$Entry<",
            "TC;TV;>;>;"
        }
    .end annotation

    .line 457
    .local p0, "this":Lcom/google/common/collect/StandardTable$Row$RowEntrySet;, "Lcom/google/common/collect/StandardTable<TR;TC;TV;>.Row.RowEntrySet;"
    iget-object v0, p0, Lcom/google/common/collect/StandardTable$Row$RowEntrySet;->this$1:Lcom/google/common/collect/StandardTable$Row;

    invoke-virtual {v0}, Lcom/google/common/collect/StandardTable$Row;->backingRowMap()Ljava/util/Map;

    move-result-object v0

    .line 458
    .local v0, "map":Ljava/util/Map;, "Ljava/util/Map<TC;TV;>;"
    if-nez v0, :cond_d

    .line 459
    invoke-static {}, Lcom/google/common/collect/Iterators;->emptyModifiableIterator()Ljava/util/Iterator;

    move-result-object v1

    return-object v1

    .line 461
    :cond_d
    invoke-interface {v0}, Ljava/util/Map;->entrySet()Ljava/util/Set;

    move-result-object v1

    invoke-interface {v1}, Ljava/util/Set;->iterator()Ljava/util/Iterator;

    move-result-object v1

    .line 462
    .local v1, "iterator":Ljava/util/Iterator;, "Ljava/util/Iterator<Ljava/util/Map$Entry<TC;TV;>;>;"
    new-instance v2, Lcom/google/common/collect/StandardTable$Row$RowEntrySet$1;

    invoke-direct {v2, p0, v1}, Lcom/google/common/collect/StandardTable$Row$RowEntrySet$1;-><init>(Lcom/google/common/collect/StandardTable$Row$RowEntrySet;Ljava/util/Iterator;)V

    return-object v2
.end method

.method map()Ljava/util/Map;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/Map<",
            "TC;TV;>;"
        }
    .end annotation

    .line 446
    .local p0, "this":Lcom/google/common/collect/StandardTable$Row$RowEntrySet;, "Lcom/google/common/collect/StandardTable<TR;TC;TV;>.Row.RowEntrySet;"
    iget-object v0, p0, Lcom/google/common/collect/StandardTable$Row$RowEntrySet;->this$1:Lcom/google/common/collect/StandardTable$Row;

    return-object v0
.end method

.method public size()I
    .registers 3

    .line 451
    .local p0, "this":Lcom/google/common/collect/StandardTable$Row$RowEntrySet;, "Lcom/google/common/collect/StandardTable<TR;TC;TV;>.Row.RowEntrySet;"
    iget-object v0, p0, Lcom/google/common/collect/StandardTable$Row$RowEntrySet;->this$1:Lcom/google/common/collect/StandardTable$Row;

    invoke-virtual {v0}, Lcom/google/common/collect/StandardTable$Row;->backingRowMap()Ljava/util/Map;

    move-result-object v0

    .line 452
    .local v0, "map":Ljava/util/Map;, "Ljava/util/Map<TC;TV;>;"
    if-nez v0, :cond_a

    const/4 v1, 0x0

    goto :goto_e

    :cond_a
    invoke-interface {v0}, Ljava/util/Map;->size()I

    move-result v1

    :goto_e
    return v1
.end method