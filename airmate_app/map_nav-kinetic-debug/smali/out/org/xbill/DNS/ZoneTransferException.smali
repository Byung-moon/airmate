.class public Lorg/xbill/DNS/ZoneTransferException;
.super Ljava/lang/Exception;
.source "ZoneTransferException.java"


# direct methods
.method public constructor <init>()V
    .registers 1

    .line 15
    invoke-direct {p0}, Ljava/lang/Exception;-><init>()V

    .line 16
    return-void
.end method

.method public constructor <init>(Ljava/lang/String;)V
    .registers 2
    .param p1, "s"    # Ljava/lang/String;

    .line 20
    invoke-direct {p0, p1}, Ljava/lang/Exception;-><init>(Ljava/lang/String;)V

    .line 21
    return-void
.end method
