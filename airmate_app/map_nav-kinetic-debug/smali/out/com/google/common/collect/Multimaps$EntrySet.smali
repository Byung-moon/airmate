.class abstract Lcom/google/common/collect/Multimaps$EntrySet;
.super Lcom/google/common/collect/Multimaps$Entries;
.source "Multimaps.java"

# interfaces
.implements Ljava/util/Set;


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/Multimaps;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x408
    name = "EntrySet"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<K:",
        "Ljava/lang/Object;",
        "V:",
        "Ljava/lang/Object;",
        ">",
        "Lcom/google/common/collect/Multimaps$Entries<",
        "TK;TV;>;",
        "Ljava/util/Set<",
        "Ljava/util/Map$Entry<",
        "TK;TV;>;>;"
    }
.end annotation


# direct methods
.method constructor <init>()V
    .registers 1

    .line 2020
    .local p0, "this":Lcom/google/common/collect/Multimaps$EntrySet;, "Lcom/google/common/collect/Multimaps$EntrySet<TK;TV;>;"
    invoke-direct {p0}, Lcom/google/common/collect/Multimaps$Entries;-><init>()V

    return-void
.end method


# virtual methods
.method public equals(Ljava/lang/Object;)Z
    .registers 3
    .param p1, "obj"    # Ljava/lang/Object;
        .annotation runtime Ljavax/annotation/Nullable;
        .end annotation
    .end param

    .line 2027
    .local p0, "this":Lcom/google/common/collect/Multimaps$EntrySet;, "Lcom/google/common/collect/Multimaps$EntrySet<TK;TV;>;"
    invoke-static {p0, p1}, Lcom/google/common/collect/Sets;->equalsImpl(Ljava/util/Set;Ljava/lang/Object;)Z

    move-result v0

    return v0
.end method

.method public hashCode()I
    .registers 2

    .line 2023
    .local p0, "this":Lcom/google/common/collect/Multimaps$EntrySet;, "Lcom/google/common/collect/Multimaps$EntrySet<TK;TV;>;"
    invoke-static {p0}, Lcom/google/common/collect/Sets;->hashCodeImpl(Ljava/util/Set;)I

    move-result v0

    return v0
.end method