.class final Lcom/google/common/base/Objects$ToStringHelper$ValueHolder;
.super Ljava/lang/Object;
.source "Objects.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/base/Objects$ToStringHelper;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x1a
    name = "ValueHolder"
.end annotation


# instance fields
.field final builder:Ljava/lang/StringBuilder;

.field isNull:Z


# direct methods
.method private constructor <init>()V
    .registers 2

    .line 435
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 436
    new-instance v0, Ljava/lang/StringBuilder;

    invoke-direct {v0}, Ljava/lang/StringBuilder;-><init>()V

    iput-object v0, p0, Lcom/google/common/base/Objects$ToStringHelper$ValueHolder;->builder:Ljava/lang/StringBuilder;

    return-void
.end method

.method synthetic constructor <init>(Lcom/google/common/base/Objects$1;)V
    .registers 2
    .param p1, "x0"    # Lcom/google/common/base/Objects$1;

    .line 435
    invoke-direct {p0}, Lcom/google/common/base/Objects$ToStringHelper$ValueHolder;-><init>()V

    return-void
.end method
