.class Lorg/apache/commons/logging/impl/WeakHashtable$1;
.super Ljava/lang/Object;
.source "WeakHashtable.java"

# interfaces
.implements Ljava/util/Enumeration;


# instance fields
.field private final synthetic this$0:Lorg/apache/commons/logging/impl/WeakHashtable;

.field private final synthetic val$enumer:Ljava/util/Enumeration;


# direct methods
.method constructor <init>(Lorg/apache/commons/logging/impl/WeakHashtable;Ljava/util/Enumeration;)V
    .registers 3
    .param p1, "this$0"    # Lorg/apache/commons/logging/impl/WeakHashtable;
    .param p2, "val$enumer"    # Ljava/util/Enumeration;

    .line 193
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    iput-object p1, p0, Lorg/apache/commons/logging/impl/WeakHashtable$1;->this$0:Lorg/apache/commons/logging/impl/WeakHashtable;

    iput-object p2, p0, Lorg/apache/commons/logging/impl/WeakHashtable$1;->val$enumer:Ljava/util/Enumeration;

    return-void
.end method


# virtual methods
.method public hasMoreElements()Z
    .registers 2

    .line 191
    iget-object v0, p0, Lorg/apache/commons/logging/impl/WeakHashtable$1;->val$enumer:Ljava/util/Enumeration;

    invoke-interface {v0}, Ljava/util/Enumeration;->hasMoreElements()Z

    move-result v0

    return v0
.end method

.method public nextElement()Ljava/lang/Object;
    .registers 3

    .line 194
    iget-object v0, p0, Lorg/apache/commons/logging/impl/WeakHashtable$1;->val$enumer:Ljava/util/Enumeration;

    invoke-interface {v0}, Ljava/util/Enumeration;->nextElement()Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Lorg/apache/commons/logging/impl/WeakHashtable$Referenced;

    .line 195
    .local v0, "nextReference":Lorg/apache/commons/logging/impl/WeakHashtable$Referenced;
    invoke-static {v0}, Lorg/apache/commons/logging/impl/WeakHashtable$Referenced;->access$100(Lorg/apache/commons/logging/impl/WeakHashtable$Referenced;)Ljava/lang/Object;

    move-result-object v1

    return-object v1
.end method
