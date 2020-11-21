.class public final Lcom/google/common/collect/MinMaxPriorityQueue$Builder;
.super Ljava/lang/Object;
.source "MinMaxPriorityQueue.java"


# annotations
.annotation build Lcom/google/common/annotations/Beta;
.end annotation

.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/MinMaxPriorityQueue;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x19
    name = "Builder"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<B:",
        "Ljava/lang/Object;",
        ">",
        "Ljava/lang/Object;"
    }
.end annotation


# static fields
.field private static final UNSET_EXPECTED_SIZE:I = -0x1


# instance fields
.field private final comparator:Ljava/util/Comparator;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/Comparator<",
            "TB;>;"
        }
    .end annotation
.end field

.field private expectedSize:I

.field private maximumSize:I


# direct methods
.method private constructor <init>(Ljava/util/Comparator;)V
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/util/Comparator<",
            "TB;>;)V"
        }
    .end annotation

    .line 165
    .local p0, "this":Lcom/google/common/collect/MinMaxPriorityQueue$Builder;, "Lcom/google/common/collect/MinMaxPriorityQueue$Builder<TB;>;"
    .local p1, "comparator":Ljava/util/Comparator;, "Ljava/util/Comparator<TB;>;"
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 162
    const/4 v0, -0x1

    iput v0, p0, Lcom/google/common/collect/MinMaxPriorityQueue$Builder;->expectedSize:I

    .line 163
    const v0, 0x7fffffff

    iput v0, p0, Lcom/google/common/collect/MinMaxPriorityQueue$Builder;->maximumSize:I

    .line 166
    invoke-static {p1}, Lcom/google/common/base/Preconditions;->checkNotNull(Ljava/lang/Object;)Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/util/Comparator;

    iput-object v0, p0, Lcom/google/common/collect/MinMaxPriorityQueue$Builder;->comparator:Ljava/util/Comparator;

    .line 167
    return-void
.end method

.method synthetic constructor <init>(Ljava/util/Comparator;Lcom/google/common/collect/MinMaxPriorityQueue$1;)V
    .registers 3
    .param p1, "x0"    # Ljava/util/Comparator;
    .param p2, "x1"    # Lcom/google/common/collect/MinMaxPriorityQueue$1;

    .line 154
    .local p0, "this":Lcom/google/common/collect/MinMaxPriorityQueue$Builder;, "Lcom/google/common/collect/MinMaxPriorityQueue$Builder<TB;>;"
    invoke-direct {p0, p1}, Lcom/google/common/collect/MinMaxPriorityQueue$Builder;-><init>(Ljava/util/Comparator;)V

    return-void
.end method

.method static synthetic access$200(Lcom/google/common/collect/MinMaxPriorityQueue$Builder;)Lcom/google/common/collect/Ordering;
    .registers 2
    .param p0, "x0"    # Lcom/google/common/collect/MinMaxPriorityQueue$Builder;

    .line 154
    invoke-direct {p0}, Lcom/google/common/collect/MinMaxPriorityQueue$Builder;->ordering()Lcom/google/common/collect/Ordering;

    move-result-object v0

    return-object v0
.end method

.method static synthetic access$300(Lcom/google/common/collect/MinMaxPriorityQueue$Builder;)I
    .registers 2
    .param p0, "x0"    # Lcom/google/common/collect/MinMaxPriorityQueue$Builder;

    .line 154
    iget v0, p0, Lcom/google/common/collect/MinMaxPriorityQueue$Builder;->maximumSize:I

    return v0
.end method

.method private ordering()Lcom/google/common/collect/Ordering;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<T:TB;>()",
            "Lcom/google/common/collect/Ordering<",
            "TT;>;"
        }
    .end annotation

    .line 215
    .local p0, "this":Lcom/google/common/collect/MinMaxPriorityQueue$Builder;, "Lcom/google/common/collect/MinMaxPriorityQueue$Builder<TB;>;"
    iget-object v0, p0, Lcom/google/common/collect/MinMaxPriorityQueue$Builder;->comparator:Ljava/util/Comparator;

    invoke-static {v0}, Lcom/google/common/collect/Ordering;->from(Ljava/util/Comparator;)Lcom/google/common/collect/Ordering;

    move-result-object v0

    return-object v0
.end method


# virtual methods
.method public create()Lcom/google/common/collect/MinMaxPriorityQueue;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<T:TB;>()",
            "Lcom/google/common/collect/MinMaxPriorityQueue<",
            "TT;>;"
        }
    .end annotation

    .line 196
    .local p0, "this":Lcom/google/common/collect/MinMaxPriorityQueue$Builder;, "Lcom/google/common/collect/MinMaxPriorityQueue$Builder<TB;>;"
    invoke-static {}, Ljava/util/Collections;->emptySet()Ljava/util/Set;

    move-result-object v0

    invoke-virtual {p0, v0}, Lcom/google/common/collect/MinMaxPriorityQueue$Builder;->create(Ljava/lang/Iterable;)Lcom/google/common/collect/MinMaxPriorityQueue;

    move-result-object v0

    return-object v0
.end method

.method public create(Ljava/lang/Iterable;)Lcom/google/common/collect/MinMaxPriorityQueue;
    .registers 5
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<T:TB;>(",
            "Ljava/lang/Iterable<",
            "+TT;>;)",
            "Lcom/google/common/collect/MinMaxPriorityQueue<",
            "TT;>;"
        }
    .end annotation

    .line 205
    .local p0, "this":Lcom/google/common/collect/MinMaxPriorityQueue$Builder;, "Lcom/google/common/collect/MinMaxPriorityQueue$Builder<TB;>;"
    .local p1, "initialContents":Ljava/lang/Iterable;, "Ljava/lang/Iterable<+TT;>;"
    new-instance v0, Lcom/google/common/collect/MinMaxPriorityQueue;

    iget v1, p0, Lcom/google/common/collect/MinMaxPriorityQueue$Builder;->expectedSize:I

    iget v2, p0, Lcom/google/common/collect/MinMaxPriorityQueue$Builder;->maximumSize:I

    invoke-static {v1, v2, p1}, Lcom/google/common/collect/MinMaxPriorityQueue;->initialQueueSize(IILjava/lang/Iterable;)I

    move-result v1

    const/4 v2, 0x0

    invoke-direct {v0, p0, v1, v2}, Lcom/google/common/collect/MinMaxPriorityQueue;-><init>(Lcom/google/common/collect/MinMaxPriorityQueue$Builder;ILcom/google/common/collect/MinMaxPriorityQueue$1;)V

    .line 207
    .local v0, "queue":Lcom/google/common/collect/MinMaxPriorityQueue;, "Lcom/google/common/collect/MinMaxPriorityQueue<TT;>;"
    invoke-interface {p1}, Ljava/lang/Iterable;->iterator()Ljava/util/Iterator;

    move-result-object v1

    .local v1, "i$":Ljava/util/Iterator;
    :goto_12
    invoke-interface {v1}, Ljava/util/Iterator;->hasNext()Z

    move-result v2

    if-eqz v2, :cond_20

    invoke-interface {v1}, Ljava/util/Iterator;->next()Ljava/lang/Object;

    move-result-object v2

    .line 208
    .local v2, "element":Ljava/lang/Object;, "TT;"
    invoke-virtual {v0, v2}, Lcom/google/common/collect/MinMaxPriorityQueue;->offer(Ljava/lang/Object;)Z

    goto :goto_12

    .line 210
    .end local v1    # "i$":Ljava/util/Iterator;
    .end local v2    # "element":Ljava/lang/Object;, "TT;"
    :cond_20
    return-object v0
.end method

.method public expectedSize(I)Lcom/google/common/collect/MinMaxPriorityQueue$Builder;
    .registers 3
    .param p1, "expectedSize"    # I
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(I)",
            "Lcom/google/common/collect/MinMaxPriorityQueue$Builder<",
            "TB;>;"
        }
    .end annotation

    .line 174
    .local p0, "this":Lcom/google/common/collect/MinMaxPriorityQueue$Builder;, "Lcom/google/common/collect/MinMaxPriorityQueue$Builder<TB;>;"
    if-ltz p1, :cond_4

    const/4 v0, 0x1

    goto :goto_5

    :cond_4
    const/4 v0, 0x0

    :goto_5
    invoke-static {v0}, Lcom/google/common/base/Preconditions;->checkArgument(Z)V

    .line 175
    iput p1, p0, Lcom/google/common/collect/MinMaxPriorityQueue$Builder;->expectedSize:I

    .line 176
    return-object p0
.end method

.method public maximumSize(I)Lcom/google/common/collect/MinMaxPriorityQueue$Builder;
    .registers 3
    .param p1, "maximumSize"    # I
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(I)",
            "Lcom/google/common/collect/MinMaxPriorityQueue$Builder<",
            "TB;>;"
        }
    .end annotation

    .line 186
    .local p0, "this":Lcom/google/common/collect/MinMaxPriorityQueue$Builder;, "Lcom/google/common/collect/MinMaxPriorityQueue$Builder<TB;>;"
    if-lez p1, :cond_4

    const/4 v0, 0x1

    goto :goto_5

    :cond_4
    const/4 v0, 0x0

    :goto_5
    invoke-static {v0}, Lcom/google/common/base/Preconditions;->checkArgument(Z)V

    .line 187
    iput p1, p0, Lcom/google/common/collect/MinMaxPriorityQueue$Builder;->maximumSize:I

    .line 188
    return-object p0
.end method
