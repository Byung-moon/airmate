.class final Lcom/google/common/collect/RegularImmutableSet;
.super Lcom/google/common/collect/ImmutableSet$ArrayImmutableSet;
.source "RegularImmutableSet.java"


# annotations
.annotation build Lcom/google/common/annotations/GwtCompatible;
    emulated = true
    serializable = true
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<E:",
        "Ljava/lang/Object;",
        ">",
        "Lcom/google/common/collect/ImmutableSet$ArrayImmutableSet<",
        "TE;>;"
    }
.end annotation


# instance fields
.field private final transient hashCode:I

.field private final transient mask:I

.field final transient table:[Ljava/lang/Object;
    .annotation build Lcom/google/common/annotations/VisibleForTesting;
    .end annotation
.end field


# direct methods
.method constructor <init>([Ljava/lang/Object;I[Ljava/lang/Object;I)V
    .registers 5
    .param p1, "elements"    # [Ljava/lang/Object;
    .param p2, "hashCode"    # I
    .param p3, "table"    # [Ljava/lang/Object;
    .param p4, "mask"    # I

    .line 39
    .local p0, "this":Lcom/google/common/collect/RegularImmutableSet;, "Lcom/google/common/collect/RegularImmutableSet<TE;>;"
    invoke-direct {p0, p1}, Lcom/google/common/collect/ImmutableSet$ArrayImmutableSet;-><init>([Ljava/lang/Object;)V

    .line 40
    iput-object p3, p0, Lcom/google/common/collect/RegularImmutableSet;->table:[Ljava/lang/Object;

    .line 41
    iput p4, p0, Lcom/google/common/collect/RegularImmutableSet;->mask:I

    .line 42
    iput p2, p0, Lcom/google/common/collect/RegularImmutableSet;->hashCode:I

    .line 43
    return-void
.end method


# virtual methods
.method public contains(Ljava/lang/Object;)Z
    .registers 6
    .param p1, "target"    # Ljava/lang/Object;

    .line 46
    .local p0, "this":Lcom/google/common/collect/RegularImmutableSet;, "Lcom/google/common/collect/RegularImmutableSet<TE;>;"
    const/4 v0, 0x0

    if-nez p1, :cond_4

    .line 47
    return v0

    .line 49
    :cond_4
    invoke-virtual {p1}, Ljava/lang/Object;->hashCode()I

    move-result v1

    invoke-static {v1}, Lcom/google/common/collect/Hashing;->smear(I)I

    move-result v1

    .line 50
    .local v1, "i":I
    :goto_c
    iget-object v2, p0, Lcom/google/common/collect/RegularImmutableSet;->table:[Ljava/lang/Object;

    iget v3, p0, Lcom/google/common/collect/RegularImmutableSet;->mask:I

    and-int/2addr v3, v1

    aget-object v2, v2, v3

    .line 51
    .local v2, "candidate":Ljava/lang/Object;
    if-nez v2, :cond_16

    .line 52
    return v0

    .line 54
    :cond_16
    invoke-virtual {v2, p1}, Ljava/lang/Object;->equals(Ljava/lang/Object;)Z

    move-result v3

    if-eqz v3, :cond_1e

    .line 55
    const/4 v0, 0x1

    return v0

    .line 49
    .end local v2    # "candidate":Ljava/lang/Object;
    :cond_1e
    add-int/lit8 v1, v1, 0x1

    goto :goto_c
.end method

.method public hashCode()I
    .registers 2

    .line 61
    .local p0, "this":Lcom/google/common/collect/RegularImmutableSet;, "Lcom/google/common/collect/RegularImmutableSet<TE;>;"
    iget v0, p0, Lcom/google/common/collect/RegularImmutableSet;->hashCode:I

    return v0
.end method

.method isHashCodeFast()Z
    .registers 2

    .line 65
    .local p0, "this":Lcom/google/common/collect/RegularImmutableSet;, "Lcom/google/common/collect/RegularImmutableSet<TE;>;"
    const/4 v0, 0x1

    return v0
.end method