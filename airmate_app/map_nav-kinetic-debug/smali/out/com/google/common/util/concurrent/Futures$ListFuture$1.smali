.class Lcom/google/common/util/concurrent/Futures$ListFuture$1;
.super Ljava/lang/Object;
.source "Futures.java"

# interfaces
.implements Ljava/lang/Runnable;


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Lcom/google/common/util/concurrent/Futures$ListFuture;->init(Ljava/util/concurrent/Executor;)V
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x0
    name = null
.end annotation


# instance fields
.field final synthetic this$0:Lcom/google/common/util/concurrent/Futures$ListFuture;


# direct methods
.method constructor <init>(Lcom/google/common/util/concurrent/Futures$ListFuture;)V
    .registers 2

    .line 1097
    .local p0, "this":Lcom/google/common/util/concurrent/Futures$ListFuture$1;, "Lcom/google/common/util/concurrent/Futures$ListFuture.1;"
    iput-object p1, p0, Lcom/google/common/util/concurrent/Futures$ListFuture$1;->this$0:Lcom/google/common/util/concurrent/Futures$ListFuture;

    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public run()V
    .registers 3

    .line 1102
    .local p0, "this":Lcom/google/common/util/concurrent/Futures$ListFuture$1;, "Lcom/google/common/util/concurrent/Futures$ListFuture.1;"
    iget-object v0, p0, Lcom/google/common/util/concurrent/Futures$ListFuture$1;->this$0:Lcom/google/common/util/concurrent/Futures$ListFuture;

    const/4 v1, 0x0

    iput-object v1, v0, Lcom/google/common/util/concurrent/Futures$ListFuture;->values:Ljava/util/List;

    .line 1105
    iget-object v0, p0, Lcom/google/common/util/concurrent/Futures$ListFuture$1;->this$0:Lcom/google/common/util/concurrent/Futures$ListFuture;

    iput-object v1, v0, Lcom/google/common/util/concurrent/Futures$ListFuture;->futures:Lcom/google/common/collect/ImmutableList;

    .line 1106
    return-void
.end method
