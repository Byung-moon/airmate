.class Landroid/support/v4/app/ActivityOptionsCompat$ActivityOptionsCompatApi23Impl;
.super Landroid/support/v4/app/ActivityOptionsCompat$ActivityOptionsCompatApi16Impl;
.source "ActivityOptionsCompat.java"


# annotations
.annotation build Landroid/support/annotation/RequiresApi;
    value = 0x17
.end annotation

.annotation system Ldalvik/annotation/EnclosingClass;
    value = Landroid/support/v4/app/ActivityOptionsCompat;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0xa
    name = "ActivityOptionsCompatApi23Impl"
.end annotation


# direct methods
.method constructor <init>(Landroid/app/ActivityOptions;)V
    .registers 2
    .param p1, "activityOptions"    # Landroid/app/ActivityOptions;

    .line 286
    invoke-direct {p0, p1}, Landroid/support/v4/app/ActivityOptionsCompat$ActivityOptionsCompatApi16Impl;-><init>(Landroid/app/ActivityOptions;)V

    .line 287
    return-void
.end method


# virtual methods
.method public requestUsageTimeReport(Landroid/app/PendingIntent;)V
    .registers 3
    .param p1, "receiver"    # Landroid/app/PendingIntent;

    .line 291
    iget-object v0, p0, Landroid/support/v4/app/ActivityOptionsCompat$ActivityOptionsCompatApi23Impl;->mActivityOptions:Landroid/app/ActivityOptions;

    invoke-virtual {v0, p1}, Landroid/app/ActivityOptions;->requestUsageTimeReport(Landroid/app/PendingIntent;)V

    .line 292
    return-void
.end method