.class final Lcom/google/common/cache/LocalCache$WeightedStrongValueReference;
.super Lcom/google/common/cache/LocalCache$StrongValueReference;
.source "LocalCache.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/cache/LocalCache;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x18
    name = "WeightedStrongValueReference"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<K:",
        "Ljava/lang/Object;",
        "V:",
        "Ljava/lang/Object;",
        ">",
        "Lcom/google/common/cache/LocalCache$StrongValueReference<",
        "TK;TV;>;"
    }
.end annotation


# instance fields
.field final weight:I


# direct methods
.method constructor <init>(Ljava/lang/Object;I)V
    .registers 3
    .param p2, "weight"    # I
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TV;I)V"
        }
    .end annotation

    .line 1835
    .local p0, "this":Lcom/google/common/cache/LocalCache$WeightedStrongValueReference;, "Lcom/google/common/cache/LocalCache$WeightedStrongValueReference<TK;TV;>;"
    .local p1, "referent":Ljava/lang/Object;, "TV;"
    invoke-direct {p0, p1}, Lcom/google/common/cache/LocalCache$StrongValueReference;-><init>(Ljava/lang/Object;)V

    .line 1836
    iput p2, p0, Lcom/google/common/cache/LocalCache$WeightedStrongValueReference;->weight:I

    .line 1837
    return-void
.end method


# virtual methods
.method public getWeight()I
    .registers 2

    .line 1841
    .local p0, "this":Lcom/google/common/cache/LocalCache$WeightedStrongValueReference;, "Lcom/google/common/cache/LocalCache$WeightedStrongValueReference<TK;TV;>;"
    iget v0, p0, Lcom/google/common/cache/LocalCache$WeightedStrongValueReference;->weight:I

    return v0
.end method
