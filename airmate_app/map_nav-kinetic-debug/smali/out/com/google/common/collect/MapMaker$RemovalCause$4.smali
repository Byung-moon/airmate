.class final enum Lcom/google/common/collect/MapMaker$RemovalCause$4;
.super Lcom/google/common/collect/MapMaker$RemovalCause;
.source "MapMaker.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/collect/MapMaker$RemovalCause;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x4008
    name = null
.end annotation


# direct methods
.method constructor <init>(Ljava/lang/String;I)V
    .registers 4

    .line 821
    const/4 v0, 0x0

    invoke-direct {p0, p1, p2, v0}, Lcom/google/common/collect/MapMaker$RemovalCause;-><init>(Ljava/lang/String;ILcom/google/common/collect/MapMaker$1;)V

    return-void
.end method


# virtual methods
.method wasEvicted()Z
    .registers 2

    .line 824
    const/4 v0, 0x1

    return v0
.end method
