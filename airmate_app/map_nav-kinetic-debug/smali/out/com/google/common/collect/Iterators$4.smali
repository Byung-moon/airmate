.class final Lcom/google/common/collect/Iterators$4;
.super Lcom/google/common/collect/UnmodifiableListIterator;
.source "Iterators.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Lcom/google/common/collect/Iterators;->unmodifiableListIterator(Ljava/util/ListIterator;)Lcom/google/common/collect/UnmodifiableListIterator;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x8
    name = null
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Lcom/google/common/collect/UnmodifiableListIterator<",
        "TT;>;"
    }
.end annotation


# instance fields
.field final synthetic val$iterator:Ljava/util/ListIterator;


# direct methods
.method constructor <init>(Ljava/util/ListIterator;)V
    .registers 2

    .line 156
    iput-object p1, p0, Lcom/google/common/collect/Iterators$4;->val$iterator:Ljava/util/ListIterator;

    invoke-direct {p0}, Lcom/google/common/collect/UnmodifiableListIterator;-><init>()V

    return-void
.end method


# virtual methods
.method public hasNext()Z
    .registers 2

    .line 159
    iget-object v0, p0, Lcom/google/common/collect/Iterators$4;->val$iterator:Ljava/util/ListIterator;

    invoke-interface {v0}, Ljava/util/ListIterator;->hasNext()Z

    move-result v0

    return v0
.end method

.method public hasPrevious()Z
    .registers 2

    .line 163
    iget-object v0, p0, Lcom/google/common/collect/Iterators$4;->val$iterator:Ljava/util/ListIterator;

    invoke-interface {v0}, Ljava/util/ListIterator;->hasPrevious()Z

    move-result v0

    return v0
.end method

.method public next()Ljava/lang/Object;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()TT;"
        }
    .end annotation

    .line 167
    iget-object v0, p0, Lcom/google/common/collect/Iterators$4;->val$iterator:Ljava/util/ListIterator;

    invoke-interface {v0}, Ljava/util/ListIterator;->next()Ljava/lang/Object;

    move-result-object v0

    return-object v0
.end method

.method public nextIndex()I
    .registers 2

    .line 175
    iget-object v0, p0, Lcom/google/common/collect/Iterators$4;->val$iterator:Ljava/util/ListIterator;

    invoke-interface {v0}, Ljava/util/ListIterator;->nextIndex()I

    move-result v0

    return v0
.end method

.method public previous()Ljava/lang/Object;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()TT;"
        }
    .end annotation

    .line 171
    iget-object v0, p0, Lcom/google/common/collect/Iterators$4;->val$iterator:Ljava/util/ListIterator;

    invoke-interface {v0}, Ljava/util/ListIterator;->previous()Ljava/lang/Object;

    move-result-object v0

    return-object v0
.end method

.method public previousIndex()I
    .registers 2

    .line 179
    iget-object v0, p0, Lcom/google/common/collect/Iterators$4;->val$iterator:Ljava/util/ListIterator;

    invoke-interface {v0}, Ljava/util/ListIterator;->previousIndex()I

    move-result v0

    return v0
.end method
