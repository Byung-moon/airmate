.class Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;
.super Landroid/graphics/drawable/Drawable$ConstantState;
.source "AnimatedVectorDrawableCompat.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0xa
    name = "AnimatedVectorDrawableCompatState"
.end annotation


# instance fields
.field mAnimatorSet:Landroid/animation/AnimatorSet;

.field private mAnimators:Ljava/util/ArrayList;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/ArrayList<",
            "Landroid/animation/Animator;",
            ">;"
        }
    .end annotation
.end field

.field mChangingConfigurations:I

.field mTargetNameMap:Landroid/support/v4/util/ArrayMap;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Landroid/support/v4/util/ArrayMap<",
            "Landroid/animation/Animator;",
            "Ljava/lang/String;",
            ">;"
        }
    .end annotation
.end field

.field mVectorDrawable:Landroid/support/graphics/drawable/VectorDrawableCompat;


# direct methods
.method public constructor <init>(Landroid/content/Context;Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;Landroid/graphics/drawable/Drawable$Callback;Landroid/content/res/Resources;)V
    .registers 12
    .param p1, "context"    # Landroid/content/Context;
    .param p2, "copy"    # Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;
    .param p3, "owner"    # Landroid/graphics/drawable/Drawable$Callback;
    .param p4, "res"    # Landroid/content/res/Resources;

    .line 585
    invoke-direct {p0}, Landroid/graphics/drawable/Drawable$ConstantState;-><init>()V

    .line 586
    if-eqz p2, :cond_8e

    .line 587
    iget v0, p2, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mChangingConfigurations:I

    iput v0, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mChangingConfigurations:I

    .line 588
    iget-object v0, p2, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mVectorDrawable:Landroid/support/graphics/drawable/VectorDrawableCompat;

    const/4 v1, 0x0

    if-eqz v0, :cond_46

    .line 589
    iget-object v0, p2, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mVectorDrawable:Landroid/support/graphics/drawable/VectorDrawableCompat;

    invoke-virtual {v0}, Landroid/support/graphics/drawable/VectorDrawableCompat;->getConstantState()Landroid/graphics/drawable/Drawable$ConstantState;

    move-result-object v0

    .line 590
    .local v0, "cs":Landroid/graphics/drawable/Drawable$ConstantState;
    if-eqz p4, :cond_1f

    .line 591
    invoke-virtual {v0, p4}, Landroid/graphics/drawable/Drawable$ConstantState;->newDrawable(Landroid/content/res/Resources;)Landroid/graphics/drawable/Drawable;

    move-result-object v2

    check-cast v2, Landroid/support/graphics/drawable/VectorDrawableCompat;

    iput-object v2, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mVectorDrawable:Landroid/support/graphics/drawable/VectorDrawableCompat;

    goto :goto_27

    .line 593
    :cond_1f
    invoke-virtual {v0}, Landroid/graphics/drawable/Drawable$ConstantState;->newDrawable()Landroid/graphics/drawable/Drawable;

    move-result-object v2

    check-cast v2, Landroid/support/graphics/drawable/VectorDrawableCompat;

    iput-object v2, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mVectorDrawable:Landroid/support/graphics/drawable/VectorDrawableCompat;

    .line 595
    :goto_27
    iget-object v2, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mVectorDrawable:Landroid/support/graphics/drawable/VectorDrawableCompat;

    invoke-virtual {v2}, Landroid/support/graphics/drawable/VectorDrawableCompat;->mutate()Landroid/graphics/drawable/Drawable;

    move-result-object v2

    check-cast v2, Landroid/support/graphics/drawable/VectorDrawableCompat;

    iput-object v2, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mVectorDrawable:Landroid/support/graphics/drawable/VectorDrawableCompat;

    .line 596
    iget-object v2, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mVectorDrawable:Landroid/support/graphics/drawable/VectorDrawableCompat;

    invoke-virtual {v2, p3}, Landroid/support/graphics/drawable/VectorDrawableCompat;->setCallback(Landroid/graphics/drawable/Drawable$Callback;)V

    .line 597
    iget-object v2, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mVectorDrawable:Landroid/support/graphics/drawable/VectorDrawableCompat;

    iget-object v3, p2, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mVectorDrawable:Landroid/support/graphics/drawable/VectorDrawableCompat;

    invoke-virtual {v3}, Landroid/support/graphics/drawable/VectorDrawableCompat;->getBounds()Landroid/graphics/Rect;

    move-result-object v3

    invoke-virtual {v2, v3}, Landroid/support/graphics/drawable/VectorDrawableCompat;->setBounds(Landroid/graphics/Rect;)V

    .line 598
    iget-object v2, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mVectorDrawable:Landroid/support/graphics/drawable/VectorDrawableCompat;

    invoke-virtual {v2, v1}, Landroid/support/graphics/drawable/VectorDrawableCompat;->setAllowCaching(Z)V

    .line 600
    .end local v0    # "cs":Landroid/graphics/drawable/Drawable$ConstantState;
    :cond_46
    iget-object v0, p2, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mAnimators:Ljava/util/ArrayList;

    if-eqz v0, :cond_8e

    .line 601
    iget-object v0, p2, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mAnimators:Ljava/util/ArrayList;

    invoke-virtual {v0}, Ljava/util/ArrayList;->size()I

    move-result v0

    .line 602
    .local v0, "numAnimators":I
    new-instance v2, Ljava/util/ArrayList;

    invoke-direct {v2, v0}, Ljava/util/ArrayList;-><init>(I)V

    iput-object v2, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mAnimators:Ljava/util/ArrayList;

    .line 603
    new-instance v2, Landroid/support/v4/util/ArrayMap;

    invoke-direct {v2, v0}, Landroid/support/v4/util/ArrayMap;-><init>(I)V

    iput-object v2, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mTargetNameMap:Landroid/support/v4/util/ArrayMap;

    .line 604
    nop

    .local v1, "i":I
    :goto_5f
    if-ge v1, v0, :cond_8b

    .line 605
    iget-object v2, p2, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mAnimators:Ljava/util/ArrayList;

    invoke-virtual {v2, v1}, Ljava/util/ArrayList;->get(I)Ljava/lang/Object;

    move-result-object v2

    check-cast v2, Landroid/animation/Animator;

    .line 606
    .local v2, "anim":Landroid/animation/Animator;
    invoke-virtual {v2}, Landroid/animation/Animator;->clone()Landroid/animation/Animator;

    move-result-object v3

    .line 607
    .local v3, "animClone":Landroid/animation/Animator;
    iget-object v4, p2, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mTargetNameMap:Landroid/support/v4/util/ArrayMap;

    invoke-virtual {v4, v2}, Landroid/support/v4/util/ArrayMap;->get(Ljava/lang/Object;)Ljava/lang/Object;

    move-result-object v4

    check-cast v4, Ljava/lang/String;

    .line 608
    .local v4, "targetName":Ljava/lang/String;
    iget-object v5, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mVectorDrawable:Landroid/support/graphics/drawable/VectorDrawableCompat;

    invoke-virtual {v5, v4}, Landroid/support/graphics/drawable/VectorDrawableCompat;->getTargetByName(Ljava/lang/String;)Ljava/lang/Object;

    move-result-object v5

    .line 609
    .local v5, "targetObject":Ljava/lang/Object;
    invoke-virtual {v3, v5}, Landroid/animation/Animator;->setTarget(Ljava/lang/Object;)V

    .line 610
    iget-object v6, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mAnimators:Ljava/util/ArrayList;

    invoke-virtual {v6, v3}, Ljava/util/ArrayList;->add(Ljava/lang/Object;)Z

    .line 611
    iget-object v6, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mTargetNameMap:Landroid/support/v4/util/ArrayMap;

    invoke-virtual {v6, v3, v4}, Landroid/support/v4/util/ArrayMap;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 604
    .end local v2    # "anim":Landroid/animation/Animator;
    .end local v3    # "animClone":Landroid/animation/Animator;
    .end local v4    # "targetName":Ljava/lang/String;
    .end local v5    # "targetObject":Ljava/lang/Object;
    add-int/lit8 v1, v1, 0x1

    goto :goto_5f

    .line 613
    .end local v1    # "i":I
    :cond_8b
    invoke-virtual {p0}, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->setupAnimatorSet()V

    .line 616
    .end local v0    # "numAnimators":I
    :cond_8e
    return-void
.end method

.method static synthetic access$000(Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;)Ljava/util/ArrayList;
    .registers 2
    .param p0, "x0"    # Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;

    .line 576
    iget-object v0, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mAnimators:Ljava/util/ArrayList;

    return-object v0
.end method

.method static synthetic access$002(Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;Ljava/util/ArrayList;)Ljava/util/ArrayList;
    .registers 2
    .param p0, "x0"    # Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;
    .param p1, "x1"    # Ljava/util/ArrayList;

    .line 576
    iput-object p1, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mAnimators:Ljava/util/ArrayList;

    return-object p1
.end method


# virtual methods
.method public getChangingConfigurations()I
    .registers 2

    .line 630
    iget v0, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mChangingConfigurations:I

    return v0
.end method

.method public newDrawable()Landroid/graphics/drawable/Drawable;
    .registers 3

    .line 620
    new-instance v0, Ljava/lang/IllegalStateException;

    const-string v1, "No constant state support for SDK < 24."

    invoke-direct {v0, v1}, Ljava/lang/IllegalStateException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public newDrawable(Landroid/content/res/Resources;)Landroid/graphics/drawable/Drawable;
    .registers 4
    .param p1, "res"    # Landroid/content/res/Resources;

    .line 625
    new-instance v0, Ljava/lang/IllegalStateException;

    const-string v1, "No constant state support for SDK < 24."

    invoke-direct {v0, v1}, Ljava/lang/IllegalStateException;-><init>(Ljava/lang/String;)V

    throw v0
.end method

.method public setupAnimatorSet()V
    .registers 3

    .line 634
    iget-object v0, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mAnimatorSet:Landroid/animation/AnimatorSet;

    if-nez v0, :cond_b

    .line 635
    new-instance v0, Landroid/animation/AnimatorSet;

    invoke-direct {v0}, Landroid/animation/AnimatorSet;-><init>()V

    iput-object v0, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mAnimatorSet:Landroid/animation/AnimatorSet;

    .line 637
    :cond_b
    iget-object v0, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mAnimatorSet:Landroid/animation/AnimatorSet;

    iget-object v1, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$AnimatedVectorDrawableCompatState;->mAnimators:Ljava/util/ArrayList;

    invoke-virtual {v0, v1}, Landroid/animation/AnimatorSet;->playTogether(Ljava/util/Collection;)V

    .line 638
    return-void
.end method
