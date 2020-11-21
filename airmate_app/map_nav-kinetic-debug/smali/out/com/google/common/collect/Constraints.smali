.class public final Lcom/google/common/collect/Constraints;
.super Ljava/lang/Object;
.source "Constraints.java"


# annotations
.annotation build Lcom/google/common/annotations/Beta;
.end annotation

.annotation build Lcom/google/common/annotations/GwtCompatible;
.end annotation

.annotation system Ldalvik/annotation/MemberClasses;
    value = {
        Lcom/google/common/collect/Constraints$ConstrainedMultiset;,
        Lcom/google/common/collect/Constraints$ConstrainedListIterator;,
        Lcom/google/common/collect/Constraints$ConstrainedRandomAccessList;,
        Lcom/google/common/collect/Constraints$ConstrainedList;,
        Lcom/google/common/collect/Constraints$ConstrainedSortedSet;,
        Lcom/google/common/collect/Constraints$ConstrainedSet;,
        Lcom/google/common/collect/Constraints$ConstrainedCollection;,
        Lcom/google/common/collect/Constraints$NotNullConstraint;
    }
.end annotation


# direct methods
.method private constructor <init>()V
    .registers 1

    .line 42
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method

.method static synthetic access$000(Ljava/util/Collection;Lcom/google/common/collect/Constraint;)Ljava/util/Collection;
    .registers 3
    .param p0, "x0"    # Ljava/util/Collection;
    .param p1, "x1"    # Lcom/google/common/collect/Constraint;

    .line 41
    invoke-static {p0, p1}, Lcom/google/common/collect/Constraints;->checkElements(Ljava/util/Collection;Lcom/google/common/collect/Constraint;)Ljava/util/Collection;

    move-result-object v0

    return-object v0
.end method

.method static synthetic access$100(Ljava/util/ListIterator;Lcom/google/common/collect/Constraint;)Ljava/util/ListIterator;
    .registers 3
    .param p0, "x0"    # Ljava/util/ListIterator;
    .param p1, "x1"    # Lcom/google/common/collect/Constraint;

    .line 41
    invoke-static {p0, p1}, Lcom/google/common/collect/Constraints;->constrainedListIterator(Ljava/util/ListIterator;Lcom/google/common/collect/Constraint;)Ljava/util/ListIterator;

    move-result-object v0

    return-object v0
.end method

.method private static checkElements(Ljava/util/Collection;Lcom/google/common/collect/Constraint;)Ljava/util/Collection;
    .registers 5
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<E:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/Collection<",
            "TE;>;",
            "Lcom/google/common/collect/Constraint<",
            "-TE;>;)",
            "Ljava/util/Collection<",
            "TE;>;"
        }
    .end annotation

    .line 376
    .local p0, "elements":Ljava/util/Collection;, "Ljava/util/Collection<TE;>;"
    .local p1, "constraint":Lcom/google/common/collect/Constraint;, "Lcom/google/common/collect/Constraint<-TE;>;"
    invoke-static {p0}, Lcom/google/common/collect/Lists;->newArrayList(Ljava/lang/Iterable;)Ljava/util/ArrayList;

    move-result-object v0

    .line 377
    .local v0, "copy":Ljava/util/Collection;, "Ljava/util/Collection<TE;>;"
    invoke-interface {v0}, Ljava/util/Collection;->iterator()Ljava/util/Iterator;

    move-result-object v1

    .local v1, "i$":Ljava/util/Iterator;
    :goto_8
    invoke-interface {v1}, Ljava/util/Iterator;->hasNext()Z

    move-result v2

    if-eqz v2, :cond_16

    invoke-interface {v1}, Ljava/util/Iterator;->next()Ljava/lang/Object;

    move-result-object v2

    .line 378
    .local v2, "element":Ljava/lang/Object;, "TE;"
    invoke-interface {p1, v2}, Lcom/google/common/collect/Constraint;->checkElement(Ljava/lang/Object;)Ljava/lang/Object;

    goto :goto_8

    .line 380
    .end local v1    # "i$":Ljava/util/Iterator;
    .end local v2    # "element":Ljava/lang/Object;, "TE;"
    :cond_16
    return-object v0
.end method

.method public static constrainedCollection(Ljava/util/Collection;Lcom/google/common/collect/Constraint;)Ljava/util/Collection;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<E:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/Collection<",
            "TE;>;",
            "Lcom/google/common/collect/Constraint<",
            "-TE;>;)",
            "Ljava/util/Collection<",
            "TE;>;"
        }
    .end annotation

    .line 82
    .local p0, "collection":Ljava/util/Collection;, "Ljava/util/Collection<TE;>;"
    .local p1, "constraint":Lcom/google/common/collect/Constraint;, "Lcom/google/common/collect/Constraint<-TE;>;"
    new-instance v0, Lcom/google/common/collect/Constraints$ConstrainedCollection;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/Constraints$ConstrainedCollection;-><init>(Ljava/util/Collection;Lcom/google/common/collect/Constraint;)V

    return-object v0
.end method

.method public static constrainedList(Ljava/util/List;Lcom/google/common/collect/Constraint;)Ljava/util/List;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<E:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/List<",
            "TE;>;",
            "Lcom/google/common/collect/Constraint<",
            "-TE;>;)",
            "Ljava/util/List<",
            "TE;>;"
        }
    .end annotation

    .line 209
    .local p0, "list":Ljava/util/List;, "Ljava/util/List<TE;>;"
    .local p1, "constraint":Lcom/google/common/collect/Constraint;, "Lcom/google/common/collect/Constraint<-TE;>;"
    instance-of v0, p0, Ljava/util/RandomAccess;

    if-eqz v0, :cond_a

    new-instance v0, Lcom/google/common/collect/Constraints$ConstrainedRandomAccessList;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/Constraints$ConstrainedRandomAccessList;-><init>(Ljava/util/List;Lcom/google/common/collect/Constraint;)V

    goto :goto_f

    :cond_a
    new-instance v0, Lcom/google/common/collect/Constraints$ConstrainedList;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/Constraints$ConstrainedList;-><init>(Ljava/util/List;Lcom/google/common/collect/Constraint;)V

    :goto_f
    return-object v0
.end method

.method private static constrainedListIterator(Ljava/util/ListIterator;Lcom/google/common/collect/Constraint;)Ljava/util/ListIterator;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<E:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/ListIterator<",
            "TE;>;",
            "Lcom/google/common/collect/Constraint<",
            "-TE;>;)",
            "Ljava/util/ListIterator<",
            "TE;>;"
        }
    .end annotation

    .line 279
    .local p0, "listIterator":Ljava/util/ListIterator;, "Ljava/util/ListIterator<TE;>;"
    .local p1, "constraint":Lcom/google/common/collect/Constraint;, "Lcom/google/common/collect/Constraint<-TE;>;"
    new-instance v0, Lcom/google/common/collect/Constraints$ConstrainedListIterator;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/Constraints$ConstrainedListIterator;-><init>(Ljava/util/ListIterator;Lcom/google/common/collect/Constraint;)V

    return-object v0
.end method

.method public static constrainedMultiset(Lcom/google/common/collect/Multiset;Lcom/google/common/collect/Constraint;)Lcom/google/common/collect/Multiset;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<E:",
            "Ljava/lang/Object;",
            ">(",
            "Lcom/google/common/collect/Multiset<",
            "TE;>;",
            "Lcom/google/common/collect/Constraint<",
            "-TE;>;)",
            "Lcom/google/common/collect/Multiset<",
            "TE;>;"
        }
    .end annotation

    .line 333
    .local p0, "multiset":Lcom/google/common/collect/Multiset;, "Lcom/google/common/collect/Multiset<TE;>;"
    .local p1, "constraint":Lcom/google/common/collect/Constraint;, "Lcom/google/common/collect/Constraint<-TE;>;"
    new-instance v0, Lcom/google/common/collect/Constraints$ConstrainedMultiset;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/Constraints$ConstrainedMultiset;-><init>(Lcom/google/common/collect/Multiset;Lcom/google/common/collect/Constraint;)V

    return-object v0
.end method

.method public static constrainedSet(Ljava/util/Set;Lcom/google/common/collect/Constraint;)Ljava/util/Set;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<E:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/Set<",
            "TE;>;",
            "Lcom/google/common/collect/Constraint<",
            "-TE;>;)",
            "Ljava/util/Set<",
            "TE;>;"
        }
    .end annotation

    .line 121
    .local p0, "set":Ljava/util/Set;, "Ljava/util/Set<TE;>;"
    .local p1, "constraint":Lcom/google/common/collect/Constraint;, "Lcom/google/common/collect/Constraint<-TE;>;"
    new-instance v0, Lcom/google/common/collect/Constraints$ConstrainedSet;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/Constraints$ConstrainedSet;-><init>(Ljava/util/Set;Lcom/google/common/collect/Constraint;)V

    return-object v0
.end method

.method public static constrainedSortedSet(Ljava/util/SortedSet;Lcom/google/common/collect/Constraint;)Ljava/util/SortedSet;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<E:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/SortedSet<",
            "TE;>;",
            "Lcom/google/common/collect/Constraint<",
            "-TE;>;)",
            "Ljava/util/SortedSet<",
            "TE;>;"
        }
    .end annotation

    .line 159
    .local p0, "sortedSet":Ljava/util/SortedSet;, "Ljava/util/SortedSet<TE;>;"
    .local p1, "constraint":Lcom/google/common/collect/Constraint;, "Lcom/google/common/collect/Constraint<-TE;>;"
    new-instance v0, Lcom/google/common/collect/Constraints$ConstrainedSortedSet;

    invoke-direct {v0, p0, p1}, Lcom/google/common/collect/Constraints$ConstrainedSortedSet;-><init>(Ljava/util/SortedSet;Lcom/google/common/collect/Constraint;)V

    return-object v0
.end method

.method static constrainedTypePreservingCollection(Ljava/util/Collection;Lcom/google/common/collect/Constraint;)Ljava/util/Collection;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<E:",
            "Ljava/lang/Object;",
            ">(",
            "Ljava/util/Collection<",
            "TE;>;",
            "Lcom/google/common/collect/Constraint<",
            "TE;>;)",
            "Ljava/util/Collection<",
            "TE;>;"
        }
    .end annotation

    .line 308
    .local p0, "collection":Ljava/util/Collection;, "Ljava/util/Collection<TE;>;"
    .local p1, "constraint":Lcom/google/common/collect/Constraint;, "Lcom/google/common/collect/Constraint<TE;>;"
    instance-of v0, p0, Ljava/util/SortedSet;

    if-eqz v0, :cond_c

    .line 309
    move-object v0, p0

    check-cast v0, Ljava/util/SortedSet;

    invoke-static {v0, p1}, Lcom/google/common/collect/Constraints;->constrainedSortedSet(Ljava/util/SortedSet;Lcom/google/common/collect/Constraint;)Ljava/util/SortedSet;

    move-result-object v0

    return-object v0

    .line 310
    :cond_c
    instance-of v0, p0, Ljava/util/Set;

    if-eqz v0, :cond_18

    .line 311
    move-object v0, p0

    check-cast v0, Ljava/util/Set;

    invoke-static {v0, p1}, Lcom/google/common/collect/Constraints;->constrainedSet(Ljava/util/Set;Lcom/google/common/collect/Constraint;)Ljava/util/Set;

    move-result-object v0

    return-object v0

    .line 312
    :cond_18
    instance-of v0, p0, Ljava/util/List;

    if-eqz v0, :cond_24

    .line 313
    move-object v0, p0

    check-cast v0, Ljava/util/List;

    invoke-static {v0, p1}, Lcom/google/common/collect/Constraints;->constrainedList(Ljava/util/List;Lcom/google/common/collect/Constraint;)Ljava/util/List;

    move-result-object v0

    return-object v0

    .line 315
    :cond_24
    invoke-static {p0, p1}, Lcom/google/common/collect/Constraints;->constrainedCollection(Ljava/util/Collection;Lcom/google/common/collect/Constraint;)Ljava/util/Collection;

    move-result-object v0

    return-object v0
.end method

.method public static notNull()Lcom/google/common/collect/Constraint;
    .registers 1
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "<E:",
            "Ljava/lang/Object;",
            ">()",
            "Lcom/google/common/collect/Constraint<",
            "TE;>;"
        }
    .end annotation

    .line 65
    sget-object v0, Lcom/google/common/collect/Constraints$NotNullConstraint;->INSTANCE:Lcom/google/common/collect/Constraints$NotNullConstraint;

    return-object v0
.end method
