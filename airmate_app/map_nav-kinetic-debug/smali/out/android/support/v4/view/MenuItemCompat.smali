.class public final Landroid/support/v4/view/MenuItemCompat;
.super Ljava/lang/Object;
.source "MenuItemCompat.java"


# annotations
.annotation system Ldalvik/annotation/MemberClasses;
    value = {
        Landroid/support/v4/view/MenuItemCompat$MenuItemCompatApi26Impl;,
        Landroid/support/v4/view/MenuItemCompat$MenuItemCompatBaseImpl;,
        Landroid/support/v4/view/MenuItemCompat$OnActionExpandListener;,
        Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;
    }
.end annotation


# static fields
.field static final IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

.field public static final SHOW_AS_ACTION_ALWAYS:I = 0x2
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation
.end field

.field public static final SHOW_AS_ACTION_COLLAPSE_ACTION_VIEW:I = 0x8
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation
.end field

.field public static final SHOW_AS_ACTION_IF_ROOM:I = 0x1
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation
.end field

.field public static final SHOW_AS_ACTION_NEVER:I = 0x0
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation
.end field

.field public static final SHOW_AS_ACTION_WITH_TEXT:I = 0x4
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation
.end field

.field private static final TAG:Ljava/lang/String; = "MenuItemCompat"


# direct methods
.method static constructor <clinit>()V
    .registers 2

    .line 276
    sget v0, Landroid/os/Build$VERSION;->SDK_INT:I

    const/16 v1, 0x1a

    if-lt v0, v1, :cond_e

    .line 277
    new-instance v0, Landroid/support/v4/view/MenuItemCompat$MenuItemCompatApi26Impl;

    invoke-direct {v0}, Landroid/support/v4/view/MenuItemCompat$MenuItemCompatApi26Impl;-><init>()V

    sput-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    goto :goto_15

    .line 279
    :cond_e
    new-instance v0, Landroid/support/v4/view/MenuItemCompat$MenuItemCompatBaseImpl;

    invoke-direct {v0}, Landroid/support/v4/view/MenuItemCompat$MenuItemCompatBaseImpl;-><init>()V

    sput-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    .line 281
    :goto_15
    return-void
.end method

.method private constructor <init>()V
    .registers 1

    .line 701
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method

.method public static collapseActionView(Landroid/view/MenuItem;)Z
    .registers 2
    .param p0, "item"    # Landroid/view/MenuItem;
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 430
    invoke-interface {p0}, Landroid/view/MenuItem;->collapseActionView()Z

    move-result v0

    return v0
.end method

.method public static expandActionView(Landroid/view/MenuItem;)Z
    .registers 2
    .param p0, "item"    # Landroid/view/MenuItem;
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 411
    invoke-interface {p0}, Landroid/view/MenuItem;->expandActionView()Z

    move-result v0

    return v0
.end method

.method public static getActionProvider(Landroid/view/MenuItem;)Landroid/support/v4/view/ActionProvider;
    .registers 3
    .param p0, "item"    # Landroid/view/MenuItem;

    .line 386
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_c

    .line 387
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0}, Landroid/support/v4/internal/view/SupportMenuItem;->getSupportActionProvider()Landroid/support/v4/view/ActionProvider;

    move-result-object v0

    return-object v0

    .line 391
    :cond_c
    const-string v0, "MenuItemCompat"

    const-string v1, "getActionProvider: item does not implement SupportMenuItem; returning null"

    invoke-static {v0, v1}, Landroid/util/Log;->w(Ljava/lang/String;Ljava/lang/String;)I

    .line 392
    const/4 v0, 0x0

    return-object v0
.end method

.method public static getActionView(Landroid/view/MenuItem;)Landroid/view/View;
    .registers 2
    .param p0, "item"    # Landroid/view/MenuItem;
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 350
    invoke-interface {p0}, Landroid/view/MenuItem;->getActionView()Landroid/view/View;

    move-result-object v0

    return-object v0
.end method

.method public static getAlphabeticModifiers(Landroid/view/MenuItem;)I
    .registers 2
    .param p0, "item"    # Landroid/view/MenuItem;

    .line 634
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_c

    .line 635
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0}, Landroid/support/v4/internal/view/SupportMenuItem;->getAlphabeticModifiers()I

    move-result v0

    return v0

    .line 637
    :cond_c
    sget-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    invoke-interface {v0, p0}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->getAlphabeticModifiers(Landroid/view/MenuItem;)I

    move-result v0

    return v0
.end method

.method public static getContentDescription(Landroid/view/MenuItem;)Ljava/lang/CharSequence;
    .registers 2
    .param p0, "item"    # Landroid/view/MenuItem;

    .line 497
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_c

    .line 498
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0}, Landroid/support/v4/internal/view/SupportMenuItem;->getContentDescription()Ljava/lang/CharSequence;

    move-result-object v0

    return-object v0

    .line 500
    :cond_c
    sget-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    invoke-interface {v0, p0}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->getContentDescription(Landroid/view/MenuItem;)Ljava/lang/CharSequence;

    move-result-object v0

    return-object v0
.end method

.method public static getIconTintList(Landroid/view/MenuItem;)Landroid/content/res/ColorStateList;
    .registers 2
    .param p0, "item"    # Landroid/view/MenuItem;

    .line 665
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_c

    .line 666
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0}, Landroid/support/v4/internal/view/SupportMenuItem;->getIconTintList()Landroid/content/res/ColorStateList;

    move-result-object v0

    return-object v0

    .line 668
    :cond_c
    sget-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    invoke-interface {v0, p0}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->getIconTintList(Landroid/view/MenuItem;)Landroid/content/res/ColorStateList;

    move-result-object v0

    return-object v0
.end method

.method public static getIconTintMode(Landroid/view/MenuItem;)Landroid/graphics/PorterDuff$Mode;
    .registers 2
    .param p0, "item"    # Landroid/view/MenuItem;

    .line 695
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_c

    .line 696
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0}, Landroid/support/v4/internal/view/SupportMenuItem;->getIconTintMode()Landroid/graphics/PorterDuff$Mode;

    move-result-object v0

    return-object v0

    .line 698
    :cond_c
    sget-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    invoke-interface {v0, p0}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->getIconTintMode(Landroid/view/MenuItem;)Landroid/graphics/PorterDuff$Mode;

    move-result-object v0

    return-object v0
.end method

.method public static getNumericModifiers(Landroid/view/MenuItem;)I
    .registers 2
    .param p0, "item"    # Landroid/view/MenuItem;

    .line 592
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_c

    .line 593
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0}, Landroid/support/v4/internal/view/SupportMenuItem;->getNumericModifiers()I

    move-result v0

    return v0

    .line 595
    :cond_c
    sget-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    invoke-interface {v0, p0}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->getNumericModifiers(Landroid/view/MenuItem;)I

    move-result v0

    return v0
.end method

.method public static getTooltipText(Landroid/view/MenuItem;)Ljava/lang/CharSequence;
    .registers 2
    .param p0, "item"    # Landroid/view/MenuItem;

    .line 523
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_c

    .line 524
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0}, Landroid/support/v4/internal/view/SupportMenuItem;->getTooltipText()Ljava/lang/CharSequence;

    move-result-object v0

    return-object v0

    .line 526
    :cond_c
    sget-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    invoke-interface {v0, p0}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->getTooltipText(Landroid/view/MenuItem;)Ljava/lang/CharSequence;

    move-result-object v0

    return-object v0
.end method

.method public static isActionViewExpanded(Landroid/view/MenuItem;)Z
    .registers 2
    .param p0, "item"    # Landroid/view/MenuItem;
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 446
    invoke-interface {p0}, Landroid/view/MenuItem;->isActionViewExpanded()Z

    move-result v0

    return v0
.end method

.method public static setActionProvider(Landroid/view/MenuItem;Landroid/support/v4/view/ActionProvider;)Landroid/view/MenuItem;
    .registers 4
    .param p0, "item"    # Landroid/view/MenuItem;
    .param p1, "provider"    # Landroid/support/v4/view/ActionProvider;

    .line 369
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_c

    .line 370
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0, p1}, Landroid/support/v4/internal/view/SupportMenuItem;->setSupportActionProvider(Landroid/support/v4/view/ActionProvider;)Landroid/support/v4/internal/view/SupportMenuItem;

    move-result-object v0

    return-object v0

    .line 373
    :cond_c
    const-string v0, "MenuItemCompat"

    const-string v1, "setActionProvider: item does not implement SupportMenuItem; ignoring"

    invoke-static {v0, v1}, Landroid/util/Log;->w(Ljava/lang/String;Ljava/lang/String;)I

    .line 374
    return-object p0
.end method

.method public static setActionView(Landroid/view/MenuItem;I)Landroid/view/MenuItem;
    .registers 3
    .param p0, "item"    # Landroid/view/MenuItem;
    .param p1, "resId"    # I
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 337
    invoke-interface {p0, p1}, Landroid/view/MenuItem;->setActionView(I)Landroid/view/MenuItem;

    move-result-object v0

    return-object v0
.end method

.method public static setActionView(Landroid/view/MenuItem;Landroid/view/View;)Landroid/view/MenuItem;
    .registers 3
    .param p0, "item"    # Landroid/view/MenuItem;
    .param p1, "view"    # Landroid/view/View;
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 315
    invoke-interface {p0, p1}, Landroid/view/MenuItem;->setActionView(Landroid/view/View;)Landroid/view/MenuItem;

    move-result-object v0

    return-object v0
.end method

.method public static setAlphabeticShortcut(Landroid/view/MenuItem;CI)V
    .registers 4
    .param p0, "item"    # Landroid/view/MenuItem;
    .param p1, "alphaChar"    # C
    .param p2, "alphaModifiers"    # I

    .line 616
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_b

    .line 617
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0, p1, p2}, Landroid/support/v4/internal/view/SupportMenuItem;->setAlphabeticShortcut(CI)Landroid/view/MenuItem;

    goto :goto_10

    .line 619
    :cond_b
    sget-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    invoke-interface {v0, p0, p1, p2}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->setAlphabeticShortcut(Landroid/view/MenuItem;CI)V

    .line 621
    :goto_10
    return-void
.end method

.method public static setContentDescription(Landroid/view/MenuItem;Ljava/lang/CharSequence;)V
    .registers 3
    .param p0, "item"    # Landroid/view/MenuItem;
    .param p1, "contentDescription"    # Ljava/lang/CharSequence;

    .line 484
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_b

    .line 485
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0, p1}, Landroid/support/v4/internal/view/SupportMenuItem;->setContentDescription(Ljava/lang/CharSequence;)Landroid/support/v4/internal/view/SupportMenuItem;

    goto :goto_10

    .line 487
    :cond_b
    sget-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    invoke-interface {v0, p0, p1}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->setContentDescription(Landroid/view/MenuItem;Ljava/lang/CharSequence;)V

    .line 489
    :goto_10
    return-void
.end method

.method public static setIconTintList(Landroid/view/MenuItem;Landroid/content/res/ColorStateList;)V
    .registers 3
    .param p0, "item"    # Landroid/view/MenuItem;
    .param p1, "tint"    # Landroid/content/res/ColorStateList;

    .line 653
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_b

    .line 654
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0, p1}, Landroid/support/v4/internal/view/SupportMenuItem;->setIconTintList(Landroid/content/res/ColorStateList;)Landroid/view/MenuItem;

    goto :goto_10

    .line 656
    :cond_b
    sget-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    invoke-interface {v0, p0, p1}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->setIconTintList(Landroid/view/MenuItem;Landroid/content/res/ColorStateList;)V

    .line 658
    :goto_10
    return-void
.end method

.method public static setIconTintMode(Landroid/view/MenuItem;Landroid/graphics/PorterDuff$Mode;)V
    .registers 3
    .param p0, "item"    # Landroid/view/MenuItem;
    .param p1, "tintMode"    # Landroid/graphics/PorterDuff$Mode;

    .line 681
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_b

    .line 682
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0, p1}, Landroid/support/v4/internal/view/SupportMenuItem;->setIconTintMode(Landroid/graphics/PorterDuff$Mode;)Landroid/view/MenuItem;

    goto :goto_10

    .line 684
    :cond_b
    sget-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    invoke-interface {v0, p0, p1}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->setIconTintMode(Landroid/view/MenuItem;Landroid/graphics/PorterDuff$Mode;)V

    .line 686
    :goto_10
    return-void
.end method

.method public static setNumericShortcut(Landroid/view/MenuItem;CI)V
    .registers 4
    .param p0, "item"    # Landroid/view/MenuItem;
    .param p1, "numericChar"    # C
    .param p2, "numericModifiers"    # I

    .line 574
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_b

    .line 575
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0, p1, p2}, Landroid/support/v4/internal/view/SupportMenuItem;->setNumericShortcut(CI)Landroid/view/MenuItem;

    goto :goto_10

    .line 577
    :cond_b
    sget-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    invoke-interface {v0, p0, p1, p2}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->setNumericShortcut(Landroid/view/MenuItem;CI)V

    .line 579
    :goto_10
    return-void
.end method

.method public static setOnActionExpandListener(Landroid/view/MenuItem;Landroid/support/v4/view/MenuItemCompat$OnActionExpandListener;)Landroid/view/MenuItem;
    .registers 3
    .param p0, "item"    # Landroid/view/MenuItem;
    .param p1, "listener"    # Landroid/support/v4/view/MenuItemCompat$OnActionExpandListener;
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 464
    new-instance v0, Landroid/support/v4/view/MenuItemCompat$1;

    invoke-direct {v0, p1}, Landroid/support/v4/view/MenuItemCompat$1;-><init>(Landroid/support/v4/view/MenuItemCompat$OnActionExpandListener;)V

    invoke-interface {p0, v0}, Landroid/view/MenuItem;->setOnActionExpandListener(Landroid/view/MenuItem$OnActionExpandListener;)Landroid/view/MenuItem;

    move-result-object v0

    return-object v0
.end method

.method public static setShortcut(Landroid/view/MenuItem;CCII)V
    .registers 12
    .param p0, "item"    # Landroid/view/MenuItem;
    .param p1, "numericChar"    # C
    .param p2, "alphaChar"    # C
    .param p3, "numericModifiers"    # I
    .param p4, "alphaModifiers"    # I

    .line 553
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_b

    .line 554
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0, p1, p2, p3, p4}, Landroid/support/v4/internal/view/SupportMenuItem;->setShortcut(CCII)Landroid/view/MenuItem;

    goto :goto_15

    .line 557
    :cond_b
    sget-object v1, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    move-object v2, p0

    move v3, p1

    move v4, p2

    move v5, p3

    move v6, p4

    invoke-interface/range {v1 .. v6}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->setShortcut(Landroid/view/MenuItem;CCII)V

    .line 559
    :goto_15
    return-void
.end method

.method public static setShowAsAction(Landroid/view/MenuItem;I)V
    .registers 2
    .param p0, "item"    # Landroid/view/MenuItem;
    .param p1, "actionEnum"    # I
    .annotation runtime Ljava/lang/Deprecated;
    .end annotation

    .line 297
    invoke-interface {p0, p1}, Landroid/view/MenuItem;->setShowAsAction(I)V

    .line 298
    return-void
.end method

.method public static setTooltipText(Landroid/view/MenuItem;Ljava/lang/CharSequence;)V
    .registers 3
    .param p0, "item"    # Landroid/view/MenuItem;
    .param p1, "tooltipText"    # Ljava/lang/CharSequence;

    .line 510
    instance-of v0, p0, Landroid/support/v4/internal/view/SupportMenuItem;

    if-eqz v0, :cond_b

    .line 511
    move-object v0, p0

    check-cast v0, Landroid/support/v4/internal/view/SupportMenuItem;

    invoke-interface {v0, p1}, Landroid/support/v4/internal/view/SupportMenuItem;->setTooltipText(Ljava/lang/CharSequence;)Landroid/support/v4/internal/view/SupportMenuItem;

    goto :goto_10

    .line 513
    :cond_b
    sget-object v0, Landroid/support/v4/view/MenuItemCompat;->IMPL:Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;

    invoke-interface {v0, p0, p1}, Landroid/support/v4/view/MenuItemCompat$MenuVersionImpl;->setTooltipText(Landroid/view/MenuItem;Ljava/lang/CharSequence;)V

    .line 515
    :goto_10
    return-void
.end method
