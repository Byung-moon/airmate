.class synthetic Lcom/google/common/collect/BstRangeOps$1;
.super Ljava/lang/Object;
.source "BstRangeOps.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/BstRangeOps;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x1008
    name = null
.end annotation


# static fields
.field static final synthetic $SwitchMap$com$google$common$collect$BstSide:[I


# direct methods
.method static constructor <clinit>()V
    .registers 3

    .line 104
    invoke-static {}, Lcom/google/common/collect/BstSide;->values()[Lcom/google/common/collect/BstSide;

    move-result-object v0

    array-length v0, v0

    new-array v0, v0, [I

    sput-object v0, Lcom/google/common/collect/BstRangeOps$1;->$SwitchMap$com$google$common$collect$BstSide:[I

    :try_start_9
    sget-object v0, Lcom/google/common/collect/BstRangeOps$1;->$SwitchMap$com$google$common$collect$BstSide:[I

    sget-object v1, Lcom/google/common/collect/BstSide;->LEFT:Lcom/google/common/collect/BstSide;

    invoke-virtual {v1}, Lcom/google/common/collect/BstSide;->ordinal()I

    move-result v1

    const/4 v2, 0x1

    aput v2, v0, v1
    :try_end_14
    .catch Ljava/lang/NoSuchFieldError; {:try_start_9 .. :try_end_14} :catch_15

    goto :goto_16

    :catch_15
    move-exception v0

    :goto_16
    :try_start_16
    sget-object v0, Lcom/google/common/collect/BstRangeOps$1;->$SwitchMap$com$google$common$collect$BstSide:[I

    sget-object v1, Lcom/google/common/collect/BstSide;->RIGHT:Lcom/google/common/collect/BstSide;

    invoke-virtual {v1}, Lcom/google/common/collect/BstSide;->ordinal()I

    move-result v1

    const/4 v2, 0x2

    aput v2, v0, v1
    :try_end_21
    .catch Ljava/lang/NoSuchFieldError; {:try_start_16 .. :try_end_21} :catch_22

    goto :goto_23

    :catch_22
    move-exception v0

    :goto_23
    return-void
.end method