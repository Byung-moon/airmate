.class abstract Lcom/google/common/collect/BstNodeFactory;
.super Ljava/lang/Object;
.source "BstNodeFactory.java"


# annotations
.annotation build Lcom/google/common/annotations/GwtCompatible;
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<N:",
        "Lcom/google/common/collect/BstNode<",
        "*TN;>;>",
        "Ljava/lang/Object;"
    }
.end annotation


# direct methods
.method constructor <init>()V
    .registers 1

    .line 32
    .local p0, "this":Lcom/google/common/collect/BstNodeFactory;, "Lcom/google/common/collect/BstNodeFactory<TN;>;"
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public final createLeaf(Lcom/google/common/collect/BstNode;)Lcom/google/common/collect/BstNode;
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TN;)TN;"
        }
    .end annotation

    .line 44
    .local p0, "this":Lcom/google/common/collect/BstNodeFactory;, "Lcom/google/common/collect/BstNodeFactory<TN;>;"
    .local p1, "source":Lcom/google/common/collect/BstNode;, "TN;"
    const/4 v0, 0x0

    invoke-virtual {p0, p1, v0, v0}, Lcom/google/common/collect/BstNodeFactory;->createNode(Lcom/google/common/collect/BstNode;Lcom/google/common/collect/BstNode;Lcom/google/common/collect/BstNode;)Lcom/google/common/collect/BstNode;

    move-result-object v0

    return-object v0
.end method

.method public abstract createNode(Lcom/google/common/collect/BstNode;Lcom/google/common/collect/BstNode;Lcom/google/common/collect/BstNode;)Lcom/google/common/collect/BstNode;
    .param p2    # Lcom/google/common/collect/BstNode;
        .annotation runtime Ljavax/annotation/Nullable;
        .end annotation
    .end param
    .param p3    # Lcom/google/common/collect/BstNode;
        .annotation runtime Ljavax/annotation/Nullable;
        .end annotation
    .end param
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TN;TN;TN;)TN;"
        }
    .end annotation
.end method
