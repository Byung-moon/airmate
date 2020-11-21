.class Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;
.super Lcom/google/common/collect/ForwardingSet;
.source "MapConstraints.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/MapConstraints;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x8
    name = "ConstrainedAsMapEntries"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<K:",
        "Ljava/lang/Object;",
        "V:",
        "Ljava/lang/Object;",
        ">",
        "Lcom/google/common/collect/ForwardingSet<",
        "Ljava/util/Map$Entry<",
        "TK;",
        "Ljava/util/Collection<",
        "TV;>;>;>;"
    }
.end annotation


# instance fields
.field private final constraint:Lcom/google/common/collect/MapConstraint;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;"
        }
    .end annotation
.end field

.field private final entries:Ljava/util/Set;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/Set<",
            "Ljava/util/Map$Entry<",
            "TK;",
            "Ljava/util/Collection<",
            "TV;>;>;>;"
        }
    .end annotation
.end field


# direct methods
.method constructor <init>(Ljava/util/Set;Lcom/google/common/collect/MapConstraint;)V
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/util/Set<",
            "Ljava/util/Map$Entry<",
            "TK;",
            "Ljava/util/Collection<",
            "TV;>;>;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)V"
        }
    .end annotation

    .line 644
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    .local p1, "entries":Ljava/util/Set;, "Ljava/util/Set<Ljava/util/Map$Entry<TK;Ljava/util/Collection<TV;>;>;>;"
    .local p2, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    invoke-direct {p0}, Lcom/google/common/collect/ForwardingSet;-><init>()V

    .line 645
    iput-object p1, p0, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->entries:Ljava/util/Set;

    .line 646
    iput-object p2, p0, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->constraint:Lcom/google/common/collect/MapConstraint;

    .line 647
    return-void
.end method

.method static synthetic access$600(Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;)Lcom/google/common/collect/MapConstraint;
    .registers 2
    .param p0, "x0"    # Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;

    .line 638
    iget-object v0, p0, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->constraint:Lcom/google/common/collect/MapConstraint;

    return-object v0
.end method


# virtual methods
.method public contains(Ljava/lang/Object;)Z
    .registers 3
    .param p1, "o"    # Ljava/lang/Object;

    .line 676
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->delegate()Ljava/util/Set;

    move-result-object v0

    invoke-static {v0, p1}, Lcom/google/common/collect/Maps;->containsEntryImpl(Ljava/util/Collection;Ljava/lang/Object;)Z

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

    .line 680
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    .local p1, "c":Ljava/util/Collection;, "Ljava/util/Collection<*>;"
    invoke-virtual {p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->standardContainsAll(Ljava/util/Collection;)Z

    move-result v0

    return v0
.end method

.method protected bridge synthetic delegate()Ljava/lang/Object;
    .registers 2

    .line 638
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->delegate()Ljava/util/Set;

    move-result-object v0

    return-object v0
.end method

.method protected bridge synthetic delegate()Ljava/util/Collection;
    .registers 2

    .line 638
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->delegate()Ljava/util/Set;

    move-result-object v0

    return-object v0
.end method

.method protected delegate()Ljava/util/Set;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/Set<",
            "Ljava/util/Map$Entry<",
            "TK;",
            "Ljava/util/Collection<",
            "TV;>;>;>;"
        }
    .end annotation

    .line 650
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    iget-object v0, p0, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->entries:Ljava/util/Set;

    return-object v0
.end method

.method public equals(Ljava/lang/Object;)Z
    .registers 3
    .param p1, "object"    # Ljava/lang/Object;
        .annotation runtime Ljavax/annotation/Nullable;
        .end annotation
    .end param

    .line 684
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    invoke-virtual {p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->standardEquals(Ljava/lang/Object;)Z

    move-result v0

    return v0
.end method

.method public hashCode()I
    .registers 2

    .line 688
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->standardHashCode()I

    move-result v0

    return v0
.end method

.method public iterator()Ljava/util/Iterator;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/Iterator<",
            "Ljava/util/Map$Entry<",
            "TK;",
            "Ljava/util/Collection<",
            "TV;>;>;>;"
        }
    .end annotation

    .line 654
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    iget-object v0, p0, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->entries:Ljava/util/Set;

    invoke-interface {v0}, Ljava/util/Set;->iterator()Ljava/util/Iterator;

    move-result-object v0

    .line 655
    .local v0, "iterator":Ljava/util/Iterator;, "Ljava/util/Iterator<Ljava/util/Map$Entry<TK;Ljava/util/Collection<TV;>;>;>;"
    new-instance v1, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries$1;

    invoke-direct {v1, p0, v0}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries$1;-><init>(Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;Ljava/util/Iterator;)V

    return-object v1
.end method

.method public remove(Ljava/lang/Object;)Z
    .registers 3
    .param p1, "o"    # Ljava/lang/Object;

    .line 692
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->delegate()Ljava/util/Set;

    move-result-object v0

    invoke-static {v0, p1}, Lcom/google/common/collect/Maps;->removeEntryImpl(Ljava/util/Collection;Ljava/lang/Object;)Z

    move-result v0

    return v0
.end method

.method public removeAll(Ljava/util/Collection;)Z
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/util/Collection<",
            "*>;)Z"
        }
    .end annotation

    .line 696
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    .local p1, "c":Ljava/util/Collection;, "Ljava/util/Collection<*>;"
    invoke-virtual {p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->standardRemoveAll(Ljava/util/Collection;)Z

    move-result v0

    return v0
.end method

.method public retainAll(Ljava/util/Collection;)Z
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/util/Collection<",
            "*>;)Z"
        }
    .end annotation

    .line 700
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    .local p1, "c":Ljava/util/Collection;, "Ljava/util/Collection<*>;"
    invoke-virtual {p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->standardRetainAll(Ljava/util/Collection;)Z

    move-result v0

    return v0
.end method

.method public toArray()[Ljava/lang/Object;
    .registers 2

    .line 668
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    invoke-virtual {p0}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->standardToArray()[Ljava/lang/Object;

    move-result-object v0

    return-object v0
.end method

.method public toArray([Ljava/lang/Object;)[Ljava/lang/Object;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<T:",
            "Ljava/lang/Object;",
            ">([TT;)[TT;"
        }
    .end annotation

    .line 672
    .local p0, "this":Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;, "Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries<TK;TV;>;"
    .local p1, "array":[Ljava/lang/Object;, "[TT;"
    invoke-virtual {p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;->standardToArray([Ljava/lang/Object;)[Ljava/lang/Object;

    move-result-object v0

    return-object v0
.end method
