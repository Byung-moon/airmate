.class Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues$1;
.super Lcom/google/common/collect/UnmodifiableIterator;
.source "Multimaps.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues;->iterator()Ljava/util/Iterator;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x0
    name = null
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Lcom/google/common/collect/UnmodifiableIterator<",
        "Ljava/util/Collection<",
        "TV;>;>;"
    }
.end annotation


# instance fields
.field final synthetic this$0:Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues;

.field final synthetic val$iterator:Ljava/util/Iterator;


# direct methods
.method constructor <init>(Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues;Ljava/util/Iterator;)V
    .registers 3

    .line 632
    .local p0, "this":Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues$1;, "Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues.1;"
    iput-object p1, p0, Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues$1;->this$0:Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues;

    iput-object p2, p0, Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues$1;->val$iterator:Ljava/util/Iterator;

    invoke-direct {p0}, Lcom/google/common/collect/UnmodifiableIterator;-><init>()V

    return-void
.end method


# virtual methods
.method public hasNext()Z
    .registers 2

    .line 635
    .local p0, "this":Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues$1;, "Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues.1;"
    iget-object v0, p0, Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues$1;->val$iterator:Ljava/util/Iterator;

    invoke-interface {v0}, Ljava/util/Iterator;->hasNext()Z

    move-result v0

    return v0
.end method

.method public bridge synthetic next()Ljava/lang/Object;
    .registers 2

    .line 632
    .local p0, "this":Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues$1;, "Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues.1;"
    invoke-virtual {p0}, Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues$1;->next()Ljava/util/Collection;

    move-result-object v0

    return-object v0
.end method

.method public next()Ljava/util/Collection;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/Collection<",
            "TV;>;"
        }
    .end annotation

    .line 639
    .local p0, "this":Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues$1;, "Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues.1;"
    iget-object v0, p0, Lcom/google/common/collect/Multimaps$UnmodifiableAsMapValues$1;->val$iterator:Ljava/util/Iterator;

    invoke-interface {v0}, Ljava/util/Iterator;->next()Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/util/Collection;

    invoke-static {v0}, Lcom/google/common/collect/Multimaps;->access$100(Ljava/util/Collection;)Ljava/util/Collection;

    move-result-object v0

    return-object v0
.end method
