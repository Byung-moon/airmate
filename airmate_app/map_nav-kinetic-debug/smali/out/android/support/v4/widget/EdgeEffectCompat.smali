.class public final Landroid/support/v4/widget/EdgeEffectCompat;
.super Ljava/lang/Object;
.source "EdgeEffectCompat.java"


# annotations
.annotation system Ldalvik/annotation/MemberClasses;
    value = {
        Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectApi21Impl;,
        Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectBaseImpl;
    }
.end annotation


# static fields
.field private static final IMPL:Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectBaseImpl;


# instance fields
.field private mEdgeEffect:Landroid/widget/EdgeEffect;


# direct methods
.method static constructor <clinit>()V
    .registers 2

    .line 39
    sget v0, Landroid/os/Build$VERSION;->SDK_INT:I

    const/16 v1, 0x15

    if-lt v0, v1, :cond_e

    .line 40
    new-instance v0, Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectApi21Impl;

    invoke-direct {v0}, Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectApi21Impl;-><init>()V

    sput-object v0, Landroid/support/v4/widget/EdgeEffectCompat;->IMPL:Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectBaseImpl;

    goto :goto_15

    .line 42
    :cond_e
    new-instance v0, Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectBaseImpl;

    invoke-direct {v0}, Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectBaseImpl;-><init>()V

    sput-object v0, Landroid/support/v4/widget/EdgeEffectCompat;->IMPL:Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectBaseImpl;

    .line 44
    :goto_15
    return-void
.end method

.method public constructor <init>(Landroid/content/Context;)V
    .registers 3
    .param p1, "context"    # Landroid/content/Context;
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 71
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 72
    new-instance v0, Landroid/widget/EdgeEffect;

    invoke-direct {v0, p1}, Landroid/widget/EdgeEffect;-><init>(Landroid/content/Context;)V

    iput-object v0, p0, Landroid/support/v4/widget/EdgeEffectCompat;->mEdgeEffect:Landroid/widget/EdgeEffect;

    .line 73
    return-void
.end method

.method public static onPull(Landroid/widget/EdgeEffect;FF)V
    .registers 4
    .param p0, "edgeEffect"    # Landroid/widget/EdgeEffect;
        .annotation build Landroid/support/annotation/NonNull;
        .end annotation
    .end param
    .param p1, "deltaDistance"    # F
    .param p2, "displacement"    # F

    .line 176
    sget-object v0, Landroid/support/v4/widget/EdgeEffectCompat;->IMPL:Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectBaseImpl;

    invoke-virtual {v0, p0, p1, p2}, Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectBaseImpl;->onPull(Landroid/widget/EdgeEffect;FF)V

    .line 177
    return-void
.end method


# virtual methods
.method public draw(Landroid/graphics/Canvas;)Z
    .registers 3
    .param p1, "canvas"    # Landroid/graphics/Canvas;
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 228
    iget-object v0, p0, Landroid/support/v4/widget/EdgeEffectCompat;->mEdgeEffect:Landroid/widget/EdgeEffect;

    invoke-virtual {v0, p1}, Landroid/widget/EdgeEffect;->draw(Landroid/graphics/Canvas;)Z

    move-result v0

    return v0
.end method

.method public finish()V
    .registers 2
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 110
    iget-object v0, p0, Landroid/support/v4/widget/EdgeEffectCompat;->mEdgeEffect:Landroid/widget/EdgeEffect;

    invoke-virtual {v0}, Landroid/widget/EdgeEffect;->finish()V

    .line 111
    return-void
.end method

.method public isFinished()Z
    .registers 2
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 99
    iget-object v0, p0, Landroid/support/v4/widget/EdgeEffectCompat;->mEdgeEffect:Landroid/widget/EdgeEffect;

    invoke-virtual {v0}, Landroid/widget/EdgeEffect;->isFinished()Z

    move-result v0

    return v0
.end method

.method public onAbsorb(I)Z
    .registers 3
    .param p1, "velocity"    # I
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 210
    iget-object v0, p0, Landroid/support/v4/widget/EdgeEffectCompat;->mEdgeEffect:Landroid/widget/EdgeEffect;

    invoke-virtual {v0, p1}, Landroid/widget/EdgeEffect;->onAbsorb(I)V

    .line 211
    const/4 v0, 0x1

    return v0
.end method

.method public onPull(F)Z
    .registers 3
    .param p1, "deltaDistance"    # F
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 128
    iget-object v0, p0, Landroid/support/v4/widget/EdgeEffectCompat;->mEdgeEffect:Landroid/widget/EdgeEffect;

    invoke-virtual {v0, p1}, Landroid/widget/EdgeEffect;->onPull(F)V

    .line 129
    const/4 v0, 0x1

    return v0
.end method

.method public onPull(FF)Z
    .registers 5
    .param p1, "deltaDistance"    # F
    .param p2, "displacement"    # F
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 153
    sget-object v0, Landroid/support/v4/widget/EdgeEffectCompat;->IMPL:Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectBaseImpl;

    iget-object v1, p0, Landroid/support/v4/widget/EdgeEffectCompat;->mEdgeEffect:Landroid/widget/EdgeEffect;

    invoke-virtual {v0, v1, p1, p2}, Landroid/support/v4/widget/EdgeEffectCompat$EdgeEffectBaseImpl;->onPull(Landroid/widget/EdgeEffect;FF)V

    .line 154
    const/4 v0, 0x1

    return v0
.end method

.method public onRelease()Z
    .registers 2
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 191
    iget-object v0, p0, Landroid/support/v4/widget/EdgeEffectCompat;->mEdgeEffect:Landroid/widget/EdgeEffect;

    invoke-virtual {v0}, Landroid/widget/EdgeEffect;->onRelease()V

    .line 192
    iget-object v0, p0, Landroid/support/v4/widget/EdgeEffectCompat;->mEdgeEffect:Landroid/widget/EdgeEffect;

    invoke-virtual {v0}, Landroid/widget/EdgeEffect;->isFinished()Z

    move-result v0

    return v0
.end method

.method public setSize(II)V
    .registers 4
    .param p1, "width"    # I
    .param p2, "height"    # I
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 85
    iget-object v0, p0, Landroid/support/v4/widget/EdgeEffectCompat;->mEdgeEffect:Landroid/widget/EdgeEffect;

    invoke-virtual {v0, p1, p2}, Landroid/widget/EdgeEffect;->setSize(II)V

    .line 86
    return-void
.end method
