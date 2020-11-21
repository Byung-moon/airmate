.class Lcom/google/common/base/Predicates$AndPredicate;
.super Ljava/lang/Object;
.source "Predicates.java"

# interfaces
.implements Lcom/google/common/base/Predicate;
.implements Ljava/io/Serializable;


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/base/Predicates;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0xa
    name = "AndPredicate"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<T:",
        "Ljava/lang/Object;",
        ">",
        "Ljava/lang/Object;",
        "Lcom/google/common/base/Predicate<",
        "TT;>;",
        "Ljava/io/Serializable;"
    }
.end annotation


# static fields
.field private static final serialVersionUID:J


# instance fields
.field private final components:Ljava/util/List;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/List<",
            "+",
            "Lcom/google/common/base/Predicate<",
            "-TT;>;>;"
        }
    .end annotation
.end field


# direct methods
.method private constructor <init>(Ljava/util/List;)V
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/util/List<",
            "+",
            "Lcom/google/common/base/Predicate<",
            "-TT;>;>;)V"
        }
    .end annotation

    .line 336
    .local p0, "this":Lcom/google/common/base/Predicates$AndPredicate;, "Lcom/google/common/base/Predicates$AndPredicate<TT;>;"
    .local p1, "components":Ljava/util/List;, "Ljava/util/List<+Lcom/google/common/base/Predicate<-TT;>;>;"
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 337
    iput-object p1, p0, Lcom/google/common/base/Predicates$AndPredicate;->components:Ljava/util/List;

    .line 338
    return-void
.end method

.method synthetic constructor <init>(Ljava/util/List;Lcom/google/common/base/Predicates$1;)V
    .registers 3
    .param p1, "x0"    # Ljava/util/List;
    .param p2, "x1"    # Lcom/google/common/base/Predicates$1;

    .line 333
    .local p0, "this":Lcom/google/common/base/Predicates$AndPredicate;, "Lcom/google/common/base/Predicates$AndPredicate<TT;>;"
    invoke-direct {p0, p1}, Lcom/google/common/base/Predicates$AndPredicate;-><init>(Ljava/util/List;)V

    return-void
.end method


# virtual methods
.method public apply(Ljava/lang/Object;)Z
    .registers 5
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TT;)Z"
        }
    .end annotation

    .line 341
    .local p0, "this":Lcom/google/common/base/Predicates$AndPredicate;, "Lcom/google/common/base/Predicates$AndPredicate<TT;>;"
    .local p1, "t":Ljava/lang/Object;, "TT;"
    const/4 v0, 0x0

    const/4 v1, 0x0

    .local v1, "i":I
    :goto_2
    iget-object v2, p0, Lcom/google/common/base/Predicates$AndPredicate;->components:Ljava/util/List;

    invoke-interface {v2}, Ljava/util/List;->size()I

    move-result v2

    if-ge v1, v2, :cond_1c

    .line 342
    iget-object v2, p0, Lcom/google/common/base/Predicates$AndPredicate;->components:Ljava/util/List;

    invoke-interface {v2, v1}, Ljava/util/List;->get(I)Ljava/lang/Object;

    move-result-object v2

    check-cast v2, Lcom/google/common/base/Predicate;

    invoke-interface {v2, p1}, Lcom/google/common/base/Predicate;->apply(Ljava/lang/Object;)Z

    move-result v2

    if-nez v2, :cond_19

    .line 343
    return v0

    .line 341
    :cond_19
    add-int/lit8 v1, v1, 0x1

    goto :goto_2

    .line 346
    .end local v1    # "i":I
    :cond_1c
    const/4 v0, 0x1

    return v0
.end method

.method public equals(Ljava/lang/Object;)Z
    .registers 5
    .param p1, "obj"    # Ljava/lang/Object;
        .annotation runtime Ljavax/annotation/Nullable;
        .end annotation
    .end param

    .line 353
    .local p0, "this":Lcom/google/common/base/Predicates$AndPredicate;, "Lcom/google/common/base/Predicates$AndPredicate<TT;>;"
    instance-of v0, p1, Lcom/google/common/base/Predicates$AndPredicate;

    if-eqz v0, :cond_10

    .line 354
    move-object v0, p1

    check-cast v0, Lcom/google/common/base/Predicates$AndPredicate;

    .line 355
    .local v0, "that":Lcom/google/common/base/Predicates$AndPredicate;, "Lcom/google/common/base/Predicates$AndPredicate<*>;"
    iget-object v1, p0, Lcom/google/common/base/Predicates$AndPredicate;->components:Ljava/util/List;

    iget-object v2, v0, Lcom/google/common/base/Predicates$AndPredicate;->components:Ljava/util/List;

    invoke-virtual {v1, v2}, Ljava/lang/Object;->equals(Ljava/lang/Object;)Z

    move-result v1

    return v1

    .line 357
    .end local v0    # "that":Lcom/google/common/base/Predicates$AndPredicate;, "Lcom/google/common/base/Predicates$AndPredicate<*>;"
    :cond_10
    const/4 v0, 0x0

    return v0
.end method

.method public hashCode()I
    .registers 3

    .line 350
    .local p0, "this":Lcom/google/common/base/Predicates$AndPredicate;, "Lcom/google/common/base/Predicates$AndPredicate<TT;>;"
    iget-object v0, p0, Lcom/google/common/base/Predicates$AndPredicate;->components:Ljava/util/List;

    invoke-virtual {v0}, Ljava/lang/Object;->hashCode()I

    move-result v0

    const v1, 0x12472c2c

    add-int/2addr v0, v1

    return v0
.end method

.method public toString()Ljava/lang/String;
    .registers 4

    .line 360
    .local p0, "this":Lcom/google/common/base/Predicates$AndPredicate;, "Lcom/google/common/base/Predicates$AndPredicate<TT;>;"
    new-instance v0, Ljava/lang/StringBuilder;

    invoke-direct {v0}, Ljava/lang/StringBuilder;-><init>()V

    const-string v1, "And("

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-static {}, Lcom/google/common/base/Predicates;->access$800()Lcom/google/common/base/Joiner;

    move-result-object v1

    iget-object v2, p0, Lcom/google/common/base/Predicates$AndPredicate;->components:Ljava/util/List;

    invoke-virtual {v1, v2}, Lcom/google/common/base/Joiner;->join(Ljava/lang/Iterable;)Ljava/lang/String;

    move-result-object v1

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v1, ")"

    invoke-virtual {v0, v1}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v0}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v0

    return-object v0
.end method
