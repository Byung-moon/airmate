.class public final Lcom/google/common/collect/MapConstraints;
.super Ljava/lang/Object;
.source "MapConstraints.java"


# annotations
.annotation build Lcom/google/common/annotations/Beta;
.end annotation

.annotation build Lcom/google/common/annotations/GwtCompatible;
.end annotation

.annotation system Ldalvik/annotation/MemberClasses;
    value = {
        Lcom/google/common/collect/MapConstraints$ConstrainedSortedSetMultimap;,
        Lcom/google/common/collect/MapConstraints$ConstrainedSetMultimap;,
        Lcom/google/common/collect/MapConstraints$ConstrainedListMultimap;,
        Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;,
        Lcom/google/common/collect/MapConstraints$ConstrainedEntrySet;,
        Lcom/google/common/collect/MapConstraints$ConstrainedEntries;,
        Lcom/google/common/collect/MapConstraints$ConstrainedAsMapValues;,
        Lcom/google/common/collect/MapConstraints$ConstrainedMultimap;,
        Lcom/google/common/collect/MapConstraints$InverseConstraint;,
        Lcom/google/common/collect/MapConstraints$ConstrainedBiMap;,
        Lcom/google/common/collect/MapConstraints$ConstrainedMap;,
        Lcom/google/common/collect/MapConstraints$NotNullMapConstraint;
    }
.end annotation


# direct methods
.method private constructor <init>()V
    .registers 1

    .line 46
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method

.method static synthetic access$000(Ljava/util/Set;Lcom/google/common/collect/MapConstraint;)Ljava/util/Set;
    .registers 3
    .param p0, "x0"    # Ljava/util/Set;
    .param p1, "x1"    # Lcom/google/common/collect/MapConstraint;

    .line 45
    invoke-static {p0, p1}, Lcom/google/common/collect/MapConstraints;->constrainedEntrySet(Ljava/util/Set;Lcom/google/common/collect/MapConstraint;)Ljava/util/Set;

    move-result-object v0

    return-object v0
.end method

.method static synthetic access$100(Ljava/util/Map;Lcom/google/common/collect/MapConstraint;)Ljava/util/Map;
    .registers 3
    .param p0, "x0"    # Ljava/util/Map;
    .param p1, "x1"    # Lcom/google/common/collect/MapConstraint;

    .line 45
    invoke-static {p0, p1}, Lcom/google/common/collect/MapConstraints;->checkMap(Ljava/util/Map;Lcom/google/common/collect/MapConstraint;)Ljava/util/Map;

    move-result-object v0

    return-object v0
.end method

.method static synthetic access$200(Ljava/util/Set;Lcom/google/common/collect/MapConstraint;)Ljava/util/Set;
    .registers 3
    .param p0, "x0"    # Ljava/util/Set;
    .param p1, "x1"    # Lcom/google/common/collect/MapConstraint;

    .line 45
    invoke-static {p0, p1}, Lcom/google/common/collect/MapConstraints;->constrainedAsMapEntries(Ljava/util/Set;Lcom/google/common/collect/MapConstraint;)Ljava/util/Set;

    move-result-object v0

    return-object v0
.end method

.method static synthetic access$300(Ljava/util/Collection;Lcom/google/common/collect/MapConstraint;)Ljava/util/Collection;
    .registers 3
    .param p0, "x0"    # Ljava/util/Collection;
    .param p1, "x1"    # Lcom/google/common/collect/MapConstraint;

    .line 45
    invoke-static {p0, p1}, Lcom/google/common/collect/MapConstraints;->constrainedEntries(Ljava/util/Collection;Lcom/google/common/collect/MapConstraint;)Ljava/util/Collection;

    move-result-object v0

    return-object v0
.end method

.method static synthetic access$400(Ljava/lang/Object;Ljava/lang/Iterable;Lcom/google/common/collect/MapConstraint;)Ljava/util/Collection;
    .registers 4
    .param p0, "x0"    # Ljava/lang/Object;
    .param p1, "x1"    # Ljava/lang/Iterable;
    .param p2, "x2"    # Lcom/google/common/collect/MapConstraint;

    .line 45
    invoke-static {p0, p1, p2}, Lcom/google/common/collect/MapConstraints;->checkValues(Ljava/lang/Object;Ljava/lang/Iterable;Lcom/google/common/collect/MapConstraint;)Ljava/util/Collection;

    move-result-object v0

    return-object v0
.end method

.method static synthetic access$500(Ljava/util/Map$Entry;Lcom/google/common/collect/MapConstraint;)Ljava/util/Map$Entry;
    .registers 3
    .param p0, "x0"    # Ljava/util/Map$Entry;
    .param p1, "x1"    # Lcom/google/common/collect/MapConstraint;

    .line 45
    invoke-static {p0, p1}, Lcom/google/common/collect/MapConstraints;->constrainedEntry(Ljava/util/Map$Entry;Lcom/google/common/collect/MapConstraint;)Ljava/util/Map$Entry;

    move-result-object v0

    return-object v0
.end method

.method static synthetic access$700(Ljava/util/Map$Entry;Lcom/google/common/collect/MapConstraint;)Ljava/util/Map$Entry;
    .registers 3
    .param p0, "x0"    # Ljava/util/Map$Entry;
    .param p1, "x1"    # Lcom/google/common/collect/MapConstraint;

    .line 45
    invoke-static {p0, p1}, Lcom/google/common/collect/MapConstraints;->constrainedAsMapEntry(Ljava/util/Map$Entry;Lcom/google/common/collect/MapConstraint;)Ljava/util/Map$Entry;

    move-result-object v0

    return-object v0
.end method

.method private static checkMap(Ljava/util/Map;Lcom/google/common/collect/MapConstraint;)Ljava/util/Map;
    .registers 7
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/Map<",
            "+TK;+TV;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Ljava/util/Map<",
            "TK;TV;>;"
        }
    .end annotation

    .line 777
    .local p0, "map":Ljava/util/Map;, "Ljava/util/Map<+TK;+TV;>;"
    .local p1, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    new-instance v0, Ljava/util/LinkedHashMap;

    invoke-direct {v0, p0}, Ljava/util/LinkedHashMap;-><init>(Ljava/util/Map;)V

    .line 778
    .local v0, "copy":Ljava/util/Map;, "Ljava/util/Map<TK;TV;>;"
    invoke-interface {v0}, Ljava/util/Map;->entrySet()Ljava/util/Set;

    move-result-object v1

    invoke-interface {v1}, Ljava/util/Set;->iterator()Ljava/util/Iterator;

    move-result-object v1

    .local v1, "i$":Ljava/util/Iterator;
    :goto_d
    invoke-interface {v1}, Ljava/util/Iterator;->hasNext()Z

    move-result v2

    if-eqz v2, :cond_25

    invoke-interface {v1}, Ljava/util/Iterator;->next()Ljava/lang/Object;

    move-result-object v2

    check-cast v2, Ljava/util/Map$Entry;

    .line 779
    .local v2, "entry":Ljava/util/Map$Entry;, "Ljava/util/Map$Entry<TK;TV;>;"
    invoke-interface {v2}, Ljava/util/Map$Entry;->getKey()Ljava/lang/Object;

    move-result-object v3

    invoke-interface {v2}, Ljava/util/Map$Entry;->getValue()Ljava/lang/Object;

    move-result-object v4

    invoke-interface {p1, v3, v4}, Lcom/google/common/collect/MapConstraint;->checkKeyValue(Ljava/lang/Object;Ljava/lang/Object;)V

    goto :goto_d

    .line 781
    .end local v1    # "i$":Ljava/util/Iterator;
    .end local v2    # "entry":Ljava/util/Map$Entry;, "Ljava/util/Map$Entry<TK;TV;>;"
    :cond_25
    return-object v0
.end method

.method private static checkValues(Ljava/lang/Object;Ljava/lang/Iterable;Lcom/google/common/collect/MapConstraint;)Ljava/util/Collection;
    .registers 6
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(TK;",
            "Ljava/lang/Iterable<",
            "+TV;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Ljava/util/Collection<",
            "TV;>;"
        }
    .end annotation

    .line 768
    .local p0, "key":Ljava/lang/Object;, "TK;"
    .local p1, "values":Ljava/lang/Iterable;, "Ljava/lang/Iterable<+TV;>;"
    .local p2, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    invoke-static {p1}, Lcom/google/common/collect/Lists;->newArrayList(Ljava/lang/Iterable;)Ljava/util/ArrayList;

    move-result-object v0

    .line 769
    .local v0, "copy":Ljava/util/Collection;, "Ljava/util/Collection<TV;>;"
    invoke-interface {v0}, Ljava/util/Collection;->iterator()Ljava/util/Iterator;

    move-result-object v1

    .local v1, "i$":Ljava/util/Iterator;
    :goto_8
    invoke-interface {v1}, Ljava/util/Iterator;->hasNext()Z

    move-result v2

    if-eqz v2, :cond_16

    invoke-interface {v1}, Ljava/util/Iterator;->next()Ljava/lang/Object;

    move-result-object v2

    .line 770
    .local v2, "value":Ljava/lang/Object;, "TV;"
    invoke-interface {p2, p0, v2}, Lcom/google/common/collect/MapConstraint;->checkKeyValue(Ljava/lang/Object;Ljava/lang/Object;)V

    goto :goto_8

    .line 772
    .end local v1    # "i$":Ljava/util/Iterator;
    .end local v2    # "value":Ljava/lang/Object;, "TV;"
    :cond_16
    return-object v0
.end method

.method private static constrainedAsMapEntries(Ljava/util/Set;Lcom/google/common/collect/MapConstraint;)Ljava/util/Set;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/Set<",
            "Ljava/util/Map$Entry<",
            "TK;",
            "Ljava/util/Collection<",
            "TV;>;>;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Ljava/util/Set<",
            "Ljava/util/Map$Entry<",
            "TK;",
            "Ljava/util/Collection<",
            "TV;>;>;>;"
        }
    .end annotation

    .line 245
    .local p0, "entries":Ljava/util/Set;, "Ljava/util/Set<Ljava/util/Map$Entry<TK;Ljava/util/Collection<TV;>;>;>;"
    .local p1, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    new-instance v0, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedAsMapEntries;-><init>(Ljava/util/Set;Lcom/google/common/collect/MapConstraint;)V

    return-object v0
.end method

.method private static constrainedAsMapEntry(Ljava/util/Map$Entry;Lcom/google/common/collect/MapConstraint;)Ljava/util/Map$Entry;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/Map$Entry<",
            "TK;",
            "Ljava/util/Collection<",
            "TV;>;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Ljava/util/Map$Entry<",
            "TK;",
            "Ljava/util/Collection<",
            "TV;>;>;"
        }
    .end annotation

    .line 211
    .local p0, "entry":Ljava/util/Map$Entry;, "Ljava/util/Map$Entry<TK;Ljava/util/Collection<TV;>;>;"
    .local p1, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    invoke-static {p0}, Lcom/google/common/base/Preconditions;->checkNotNull(Ljava/lang/Object;)Ljava/lang/Object;

    .line 212
    invoke-static {p1}, Lcom/google/common/base/Preconditions;->checkNotNull(Ljava/lang/Object;)Ljava/lang/Object;

    .line 213
    new-instance v0, Lcom/google/common/collect/MapConstraints$2;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/MapConstraints$2;-><init>(Ljava/util/Map$Entry;Lcom/google/common/collect/MapConstraint;)V

    return-object v0
.end method

.method public static constrainedBiMap(Lcom/google/common/collect/BiMap;Lcom/google/common/collect/MapConstraint;)Lcom/google/common/collect/BiMap;
    .registers 4
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(",
            "Lcom/google/common/collect/BiMap<",
            "TK;TV;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Lcom/google/common/collect/BiMap<",
            "TK;TV;>;"
        }
    .end annotation

    .line 333
    .local p0, "map":Lcom/google/common/collect/BiMap;, "Lcom/google/common/collect/BiMap<TK;TV;>;"
    .local p1, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    new-instance v0, Lcom/google/common/collect/MapConstraints$ConstrainedBiMap;

    const/4 v1, 0x0

    invoke-direct {v0, p0, v1, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedBiMap;-><init>(Lcom/google/common/collect/BiMap;Lcom/google/common/collect/BiMap;Lcom/google/common/collect/MapConstraint;)V

    return-object v0
.end method

.method private static constrainedEntries(Ljava/util/Collection;Lcom/google/common/collect/MapConstraint;)Ljava/util/Collection;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/Collection<",
            "Ljava/util/Map$Entry<",
            "TK;TV;>;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Ljava/util/Collection<",
            "Ljava/util/Map$Entry<",
            "TK;TV;>;>;"
        }
    .end annotation

    .line 263
    .local p0, "entries":Ljava/util/Collection;, "Ljava/util/Collection<Ljava/util/Map$Entry<TK;TV;>;>;"
    .local p1, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    instance-of v0, p0, Ljava/util/Set;

    if-eqz v0, :cond_c

    .line 264
    move-object v0, p0

    check-cast v0, Ljava/util/Set;

    invoke-static {v0, p1}, Lcom/google/common/collect/MapConstraints;->constrainedEntrySet(Ljava/util/Set;Lcom/google/common/collect/MapConstraint;)Ljava/util/Set;

    move-result-object v0

    return-object v0

    .line 266
    :cond_c
    new-instance v0, Lcom/google/common/collect/MapConstraints$ConstrainedEntries;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedEntries;-><init>(Ljava/util/Collection;Lcom/google/common/collect/MapConstraint;)V

    return-object v0
.end method

.method private static constrainedEntry(Ljava/util/Map$Entry;Lcom/google/common/collect/MapConstraint;)Ljava/util/Map$Entry;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/Map$Entry<",
            "TK;TV;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Ljava/util/Map$Entry<",
            "TK;TV;>;"
        }
    .end annotation

    .line 185
    .local p0, "entry":Ljava/util/Map$Entry;, "Ljava/util/Map$Entry<TK;TV;>;"
    .local p1, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    invoke-static {p0}, Lcom/google/common/base/Preconditions;->checkNotNull(Ljava/lang/Object;)Ljava/lang/Object;

    .line 186
    invoke-static {p1}, Lcom/google/common/base/Preconditions;->checkNotNull(Ljava/lang/Object;)Ljava/lang/Object;

    .line 187
    new-instance v0, Lcom/google/common/collect/MapConstraints$1;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/MapConstraints$1;-><init>(Ljava/util/Map$Entry;Lcom/google/common/collect/MapConstraint;)V

    return-object v0
.end method

.method private static constrainedEntrySet(Ljava/util/Set;Lcom/google/common/collect/MapConstraint;)Ljava/util/Set;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/Set<",
            "Ljava/util/Map$Entry<",
            "TK;TV;>;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Ljava/util/Set<",
            "Ljava/util/Map$Entry<",
            "TK;TV;>;>;"
        }
    .end annotation

    .line 286
    .local p0, "entries":Ljava/util/Set;, "Ljava/util/Set<Ljava/util/Map$Entry<TK;TV;>;>;"
    .local p1, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    new-instance v0, Lcom/google/common/collect/MapConstraints$ConstrainedEntrySet;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedEntrySet;-><init>(Ljava/util/Set;Lcom/google/common/collect/MapConstraint;)V

    return-object v0
.end method

.method public static constrainedListMultimap(Lcom/google/common/collect/ListMultimap;Lcom/google/common/collect/MapConstraint;)Lcom/google/common/collect/ListMultimap;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(",
            "Lcom/google/common/collect/ListMultimap<",
            "TK;TV;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Lcom/google/common/collect/ListMultimap<",
            "TK;TV;>;"
        }
    .end annotation

    .line 128
    .local p0, "multimap":Lcom/google/common/collect/ListMultimap;, "Lcom/google/common/collect/ListMultimap<TK;TV;>;"
    .local p1, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    new-instance v0, Lcom/google/common/collect/MapConstraints$ConstrainedListMultimap;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedListMultimap;-><init>(Lcom/google/common/collect/ListMultimap;Lcom/google/common/collect/MapConstraint;)V

    return-object v0
.end method

.method public static constrainedMap(Ljava/util/Map;Lcom/google/common/collect/MapConstraint;)Ljava/util/Map;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/Map<",
            "TK;TV;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Ljava/util/Map<",
            "TK;TV;>;"
        }
    .end annotation

    .line 85
    .local p0, "map":Ljava/util/Map;, "Ljava/util/Map<TK;TV;>;"
    .local p1, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    new-instance v0, Lcom/google/common/collect/MapConstraints$ConstrainedMap;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedMap;-><init>(Ljava/util/Map;Lcom/google/common/collect/MapConstraint;)V

    return-object v0
.end method

.method public static constrainedMultimap(Lcom/google/common/collect/Multimap;Lcom/google/common/collect/MapConstraint;)Lcom/google/common/collect/Multimap;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(",
            "Lcom/google/common/collect/Multimap<",
            "TK;TV;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Lcom/google/common/collect/Multimap<",
            "TK;TV;>;"
        }
    .end annotation

    .line 106
    .local p0, "multimap":Lcom/google/common/collect/Multimap;, "Lcom/google/common/collect/Multimap<TK;TV;>;"
    .local p1, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    new-instance v0, Lcom/google/common/collect/MapConstraints$ConstrainedMultimap;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedMultimap;-><init>(Lcom/google/common/collect/Multimap;Lcom/google/common/collect/MapConstraint;)V

    return-object v0
.end method

.method public static constrainedSetMultimap(Lcom/google/common/collect/SetMultimap;Lcom/google/common/collect/MapConstraint;)Lcom/google/common/collect/SetMultimap;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(",
            "Lcom/google/common/collect/SetMultimap<",
            "TK;TV;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Lcom/google/common/collect/SetMultimap<",
            "TK;TV;>;"
        }
    .end annotation

    .line 149
    .local p0, "multimap":Lcom/google/common/collect/SetMultimap;, "Lcom/google/common/collect/SetMultimap<TK;TV;>;"
    .local p1, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    new-instance v0, Lcom/google/common/collect/MapConstraints$ConstrainedSetMultimap;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedSetMultimap;-><init>(Lcom/google/common/collect/SetMultimap;Lcom/google/common/collect/MapConstraint;)V

    return-object v0
.end method

.method public static constrainedSortedSetMultimap(Lcom/google/common/collect/SortedSetMultimap;Lcom/google/common/collect/MapConstraint;)Lcom/google/common/collect/SortedSetMultimap;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<K:",
            "Ljava/lang/Object;",
            "V:",
            "Ljava/lang/Object;",
            ">(",
            "Lcom/google/common/collect/SortedSetMultimap<",
            "TK;TV;>;",
            "Lcom/google/common/collect/MapConstraint<",
            "-TK;-TV;>;)",
            "Lcom/google/common/collect/SortedSetMultimap<",
            "TK;TV;>;"
        }
    .end annotation

    .line 170
    .local p0, "multimap":Lcom/google/common/collect/SortedSetMultimap;, "Lcom/google/common/collect/SortedSetMultimap<TK;TV;>;"
    .local p1, "constraint":Lcom/google/common/collect/MapConstraint;, "Lcom/google/common/collect/MapConstraint<-TK;-TV;>;"
    new-instance v0, Lcom/google/common/collect/MapConstraints$ConstrainedSortedSetMultimap;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/MapConstraints$ConstrainedSortedSetMultimap;-><init>(Lcom/google/common/collect/SortedSetMultimap;Lcom/google/common/collect/MapConstraint;)V

    return-object v0
.end method

.method public static notNull()Lcom/google/common/collect/MapConstraint;
    .registers 1
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Lcom/google/common/collect/MapConstraint<",
            "Ljava/lang/Object;",
            "Ljava/lang/Object;",
            ">;"
        }
    .end annotation

    .line 53
    sget-object v0, Lcom/google/common/collect/MapConstraints$NotNullMapConstraint;->INSTANCE:Lcom/google/common/collect/MapConstraints$NotNullMapConstraint;

    return-object v0
.end method
