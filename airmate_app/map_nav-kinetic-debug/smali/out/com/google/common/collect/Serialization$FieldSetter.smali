.class final Lcom/google/common/collect/Serialization$FieldSetter;
.super Ljava/lang/Object;
.source "Serialization.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/Serialization;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x18
    name = "FieldSetter"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "<T:",
        "Ljava/lang/Object;",
        ">",
        "Ljava/lang/Object;"
    }
.end annotation


# instance fields
.field private final field:Ljava/lang/reflect/Field;


# direct methods
.method private constructor <init>(Ljava/lang/reflect/Field;)V
    .registers 3
    .param p1, "field"    # Ljava/lang/reflect/Field;

    .line 210
    .local p0, "this":Lcom/google/common/collect/Serialization$FieldSetter;, "Lcom/google/common/collect/Serialization$FieldSetter<TT;>;"
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 211
    iput-object p1, p0, Lcom/google/common/collect/Serialization$FieldSetter;->field:Ljava/lang/reflect/Field;

    .line 212
    const/4 v0, 0x1

    invoke-virtual {p1, v0}, Ljava/lang/reflect/Field;->setAccessible(Z)V

    .line 213
    return-void
.end method

.method synthetic constructor <init>(Ljava/lang/reflect/Field;Lcom/google/common/collect/Serialization$1;)V
    .registers 3
    .param p1, "x0"    # Ljava/lang/reflect/Field;
    .param p2, "x1"    # Lcom/google/common/collect/Serialization$1;

    .line 207
    .local p0, "this":Lcom/google/common/collect/Serialization$FieldSetter;, "Lcom/google/common/collect/Serialization$FieldSetter<TT;>;"
    invoke-direct {p0, p1}, Lcom/google/common/collect/Serialization$FieldSetter;-><init>(Ljava/lang/reflect/Field;)V

    return-void
.end method


# virtual methods
.method set(Ljava/lang/Object;I)V
    .registers 5
    .param p2, "value"    # I
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TT;I)V"
        }
    .end annotation

    .line 225
    .local p0, "this":Lcom/google/common/collect/Serialization$FieldSetter;, "Lcom/google/common/collect/Serialization$FieldSetter<TT;>;"
    .local p1, "instance":Ljava/lang/Object;, "TT;"
    :try_start_0
    iget-object v0, p0, Lcom/google/common/collect/Serialization$FieldSetter;->field:Ljava/lang/reflect/Field;

    invoke-static {p2}, Ljava/lang/Integer;->valueOf(I)Ljava/lang/Integer;

    move-result-object v1

    invoke-virtual {v0, p1, v1}, Ljava/lang/reflect/Field;->set(Ljava/lang/Object;Ljava/lang/Object;)V
    :try_end_9
    .catch Ljava/lang/IllegalAccessException; {:try_start_0 .. :try_end_9} :catch_b

    .line 228
    nop

    .line 229
    return-void

    .line 226
    :catch_b
    move-exception v0

    .line 227
    .local v0, "impossible":Ljava/lang/IllegalAccessException;
    new-instance v1, Ljava/lang/AssertionError;

    invoke-direct {v1, v0}, Ljava/lang/AssertionError;-><init>(Ljava/lang/Object;)V

    throw v1
.end method

.method set(Ljava/lang/Object;Ljava/lang/Object;)V
    .registers 5
    .param p2, "value"    # Ljava/lang/Object;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(TT;",
            "Ljava/lang/Object;",
            ")V"
        }
    .end annotation

    .line 217
    .local p0, "this":Lcom/google/common/collect/Serialization$FieldSetter;, "Lcom/google/common/collect/Serialization$FieldSetter<TT;>;"
    .local p1, "instance":Ljava/lang/Object;, "TT;"
    :try_start_0
    iget-object v0, p0, Lcom/google/common/collect/Serialization$FieldSetter;->field:Ljava/lang/reflect/Field;

    invoke-virtual {v0, p1, p2}, Ljava/lang/reflect/Field;->set(Ljava/lang/Object;Ljava/lang/Object;)V
    :try_end_5
    .catch Ljava/lang/IllegalAccessException; {:try_start_0 .. :try_end_5} :catch_7

    .line 220
    nop

    .line 221
    return-void

    .line 218
    :catch_7
    move-exception v0

    .line 219
    .local v0, "impossible":Ljava/lang/IllegalAccessException;
    new-instance v1, Ljava/lang/AssertionError;

    invoke-direct {v1, v0}, Ljava/lang/AssertionError;-><init>(Ljava/lang/Object;)V

    throw v1
.end method
