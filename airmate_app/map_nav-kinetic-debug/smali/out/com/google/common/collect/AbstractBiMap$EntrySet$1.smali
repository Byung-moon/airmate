.class Lcom/google/common/collect/AbstractBiMap$EntrySet$1;
.super Ljava/lang/Object;
.source "AbstractBiMap.java"

# interfaces
.implements Ljava/util/Iterator;


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Lcom/google/common/collect/AbstractBiMap$EntrySet;->iterator()Ljava/util/Iterator;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x0
    name = null
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/lang/Object;",
        "Ljava/util/Iterator<",
        "Ljava/util/Map$Entry<",
        "TK;TV;>;>;"
    }
.end annotation


# instance fields
.field entry:Ljava/util/Map$Entry;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/Map$Entry<",
            "TK;TV;>;"
        }
    .end annotation
.end field

.field final synthetic this$1:Lcom/google/common/collect/AbstractBiMap$EntrySet;

.field final synthetic val$iterator:Ljava/util/Iterator;


# direct methods
.method constructor <init>(Lcom/google/common/collect/AbstractBiMap$EntrySet;Ljava/util/Iterator;)V
    .registers 3

    .line 286
    .local p0, "this":Lcom/google/common/collect/AbstractBiMap$EntrySet$1;, "Lcom/google/common/collect/AbstractBiMap$EntrySet.1;"
    iput-object p1, p0, Lcom/google/common/collect/AbstractBiMap$EntrySet$1;->this$1:Lcom/google/common/collect/AbstractBiMap$EntrySet;

    iput-object p2, p0, Lcom/google/common/collect/AbstractBiMap$EntrySet$1;->val$iterator:Ljava/util/Iterator;

    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public hasNext()Z
    .registers 2

    .line 290
    .local p0, "this":Lcom/google/common/collect/AbstractBiMap$EntrySet$1;, "Lcom/google/common/collect/AbstractBiMap$EntrySet.1;"
    iget-object v0, p0, Lcom/google/common/collect/AbstractBiMap$EntrySet$1;->val$iterator:Ljava/util/Iterator;

    invoke-interface {v0}, Ljava/util/Iterator;->hasNext()Z

    move-result v0

    return v0
.end method

.method public bridge synthetic next()Ljava/lang/Object;
    .registers 2

    .line 286
    .local p0, "this":Lcom/google/common/collect/AbstractBiMap$EntrySet$1;, "Lcom/google/common/collect/AbstractBiMap$EntrySet.1;"
    invoke-virtual {p0}, Lcom/google/common/collect/AbstractBiMap$EntrySet$1;->next()Ljava/util/Map$Entry;

    move-result-object v0

    return-object v0
.end method

.method public next()Ljava/util/Map$Entry;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/Map$Entry<",
            "TK;TV;>;"
        }
    .end annotation

    .line 294
    .local p0, "this":Lcom/google/common/collect/AbstractBiMap$EntrySet$1;, "Lcom/google/common/collect/AbstractBiMap$EntrySet.1;"
    iget-object v0, p0, Lcom/google/common/collect/AbstractBiMap$EntrySet$1;->val$iterator:Ljava/util/Iterator;

    invoke-interface {v0}, Ljava/util/Iterator;->next()Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Ljava/util/Map$Entry;

    iput-object v0, p0, Lcom/google/common/collect/AbstractBiMap$EntrySet$1;->entry:Ljava/util/Map$Entry;

    .line 295
    iget-object v0, p0, Lcom/google/common/collect/AbstractBiMap$EntrySet$1;->entry:Ljava/util/Map$Entry;

    .line 297
    .local v0, "finalEntry":Ljava/util/Map$Entry;, "Ljava/util/Map$Entry<TK;TV;>;"
    new-instance v1, Lcom/google/common/collect/AbstractBiMap$EntrySet$1$1;

    invoke-direct {v1, p0, v0}, Lcom/google/common/collect/AbstractBiMap$EntrySet$1$1;-><init>(Lcom/google/common/collect/AbstractBiMap$EntrySet$1;Ljava/util/Map$Entry;)V

    return-object v1
.end method

.method public remove()V
    .registers 3

    .line 321
    .local p0, "this":Lcom/google/common/collect/AbstractBiMap$EntrySet$1;, "Lcom/google/common/collect/AbstractBiMap$EntrySet.1;"
    iget-object v0, p0, Lcom/google/common/collect/AbstractBiMap$EntrySet$1;->entry:Ljava/util/Map$Entry;

    if-eqz v0, :cond_6

    const/4 v0, 0x1

    goto :goto_7

    :cond_6
    const/4 v0, 0x0

    :goto_7
    invoke-static {v0}, Lcom/google/common/base/Preconditions;->checkState(Z)V

    .line 322
    iget-object v0, p0, Lcom/google/common/collect/AbstractBiMap$EntrySet$1;->entry:Ljava/util/Map$Entry;

    invoke-interface {v0}, Ljava/util/Map$Entry;->getValue()Ljava/lang/Object;

    move-result-object v0

    .line 323
    .local v0, "value":Ljava/lang/Object;, "TV;"
    iget-object v1, p0, Lcom/google/common/collect/AbstractBiMap$EntrySet$1;->val$iterator:Ljava/util/Iterator;

    invoke-interface {v1}, Ljava/util/Iterator;->remove()V

    .line 324
    iget-object v1, p0, Lcom/google/common/collect/AbstractBiMap$EntrySet$1;->this$1:Lcom/google/common/collect/AbstractBiMap$EntrySet;

    iget-object v1, v1, Lcom/google/common/collect/AbstractBiMap$EntrySet;->this$0:Lcom/google/common/collect/AbstractBiMap;

    invoke-static {v1, v0}, Lcom/google/common/collect/AbstractBiMap;->access$700(Lcom/google/common/collect/AbstractBiMap;Ljava/lang/Object;)V

    .line 325
    return-void
.end method
