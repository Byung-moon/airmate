.class Lorg/ros/internal/node/client/Registrar$7$1;
.super Ljava/lang/Object;
.source "Registrar.java"

# interfaces
.implements Ljava/util/concurrent/Callable;


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Lorg/ros/internal/node/client/Registrar$7;->call()Ljava/lang/Boolean;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x0
    name = null
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/lang/Object;",
        "Ljava/util/concurrent/Callable<",
        "Lorg/ros/internal/node/response/Response<",
        "Ljava/lang/Integer;",
        ">;>;"
    }
.end annotation


# instance fields
.field final synthetic this$1:Lorg/ros/internal/node/client/Registrar$7;


# direct methods
.method constructor <init>(Lorg/ros/internal/node/client/Registrar$7;)V
    .registers 2
    .param p1, "this$1"    # Lorg/ros/internal/node/client/Registrar$7;

    .line 236
    iput-object p1, p0, Lorg/ros/internal/node/client/Registrar$7$1;->this$1:Lorg/ros/internal/node/client/Registrar$7;

    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public bridge synthetic call()Ljava/lang/Object;
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/Exception;
        }
    .end annotation

    .line 236
    invoke-virtual {p0}, Lorg/ros/internal/node/client/Registrar$7$1;->call()Lorg/ros/internal/node/response/Response;

    move-result-object v0

    return-object v0
.end method

.method public call()Lorg/ros/internal/node/response/Response;
    .registers 4
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Lorg/ros/internal/node/response/Response<",
            "Ljava/lang/Integer;",
            ">;"
        }
    .end annotation

    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/Exception;
        }
    .end annotation

    .line 239
    iget-object v0, p0, Lorg/ros/internal/node/client/Registrar$7$1;->this$1:Lorg/ros/internal/node/client/Registrar$7;

    iget-object v0, v0, Lorg/ros/internal/node/client/Registrar$7;->this$0:Lorg/ros/internal/node/client/Registrar;

    invoke-static {v0}, Lorg/ros/internal/node/client/Registrar;->access$000(Lorg/ros/internal/node/client/Registrar;)Lorg/ros/internal/node/client/MasterClient;

    move-result-object v0

    iget-object v1, p0, Lorg/ros/internal/node/client/Registrar$7$1;->this$1:Lorg/ros/internal/node/client/Registrar$7;

    iget-object v1, v1, Lorg/ros/internal/node/client/Registrar$7;->this$0:Lorg/ros/internal/node/client/Registrar;

    invoke-static {v1}, Lorg/ros/internal/node/client/Registrar;->access$200(Lorg/ros/internal/node/client/Registrar;)Lorg/ros/internal/node/server/NodeIdentifier;

    move-result-object v1

    iget-object v2, p0, Lorg/ros/internal/node/client/Registrar$7$1;->this$1:Lorg/ros/internal/node/client/Registrar$7;

    iget-object v2, v2, Lorg/ros/internal/node/client/Registrar$7;->val$subscriber:Lorg/ros/internal/node/topic/DefaultSubscriber;

    invoke-virtual {v0, v1, v2}, Lorg/ros/internal/node/client/MasterClient;->unregisterSubscriber(Lorg/ros/internal/node/server/NodeIdentifier;Lorg/ros/node/topic/Subscriber;)Lorg/ros/internal/node/response/Response;

    move-result-object v0

    return-object v0
.end method
