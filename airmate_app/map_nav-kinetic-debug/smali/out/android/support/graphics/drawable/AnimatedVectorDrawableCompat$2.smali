.class Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$2;
.super Landroid/animation/AnimatorListenerAdapter;
.source "AnimatedVectorDrawableCompat.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;->registerAnimationCallback(Landroid/support/graphics/drawable/Animatable2Compat$AnimationCallback;)V
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x0
    name = null
.end annotation


# instance fields
.field final synthetic this$0:Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;


# direct methods
.method constructor <init>(Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;)V
    .registers 2
    .param p1, "this$0"    # Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;

    .line 774
    iput-object p1, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$2;->this$0:Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;

    invoke-direct {p0}, Landroid/animation/AnimatorListenerAdapter;-><init>()V

    return-void
.end method


# virtual methods
.method public onAnimationEnd(Landroid/animation/Animator;)V
    .registers 7
    .param p1, "animation"    # Landroid/animation/Animator;

    .line 787
    new-instance v0, Ljava/util/ArrayList;

    iget-object v1, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$2;->this$0:Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;

    .line 788
    invoke-static {v1}, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;->access$100(Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;)Ljava/util/ArrayList;

    move-result-object v1

    invoke-direct {v0, v1}, Ljava/util/ArrayList;-><init>(Ljava/util/Collection;)V

    .line 789
    .local v0, "tmpCallbacks":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/support/graphics/drawable/Animatable2Compat$AnimationCallback;>;"
    invoke-virtual {v0}, Ljava/util/ArrayList;->size()I

    move-result v1

    .line 790
    .local v1, "size":I
    const/4 v2, 0x0

    .local v2, "i":I
    :goto_10
    if-ge v2, v1, :cond_20

    .line 791
    invoke-virtual {v0, v2}, Ljava/util/ArrayList;->get(I)Ljava/lang/Object;

    move-result-object v3

    check-cast v3, Landroid/support/graphics/drawable/Animatable2Compat$AnimationCallback;

    iget-object v4, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$2;->this$0:Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;

    invoke-virtual {v3, v4}, Landroid/support/graphics/drawable/Animatable2Compat$AnimationCallback;->onAnimationEnd(Landroid/graphics/drawable/Drawable;)V

    .line 790
    add-int/lit8 v2, v2, 0x1

    goto :goto_10

    .line 793
    .end local v2    # "i":I
    :cond_20
    return-void
.end method

.method public onAnimationStart(Landroid/animation/Animator;)V
    .registers 7
    .param p1, "animation"    # Landroid/animation/Animator;

    .line 777
    new-instance v0, Ljava/util/ArrayList;

    iget-object v1, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$2;->this$0:Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;

    .line 778
    invoke-static {v1}, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;->access$100(Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;)Ljava/util/ArrayList;

    move-result-object v1

    invoke-direct {v0, v1}, Ljava/util/ArrayList;-><init>(Ljava/util/Collection;)V

    .line 779
    .local v0, "tmpCallbacks":Ljava/util/ArrayList;, "Ljava/util/ArrayList<Landroid/support/graphics/drawable/Animatable2Compat$AnimationCallback;>;"
    invoke-virtual {v0}, Ljava/util/ArrayList;->size()I

    move-result v1

    .line 780
    .local v1, "size":I
    const/4 v2, 0x0

    .local v2, "i":I
    :goto_10
    if-ge v2, v1, :cond_20

    .line 781
    invoke-virtual {v0, v2}, Ljava/util/ArrayList;->get(I)Ljava/lang/Object;

    move-result-object v3

    check-cast v3, Landroid/support/graphics/drawable/Animatable2Compat$AnimationCallback;

    iget-object v4, p0, Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat$2;->this$0:Landroid/support/graphics/drawable/AnimatedVectorDrawableCompat;

    invoke-virtual {v3, v4}, Landroid/support/graphics/drawable/Animatable2Compat$AnimationCallback;->onAnimationStart(Landroid/graphics/drawable/Drawable;)V

    .line 780
    add-int/lit8 v2, v2, 0x1

    goto :goto_10

    .line 783
    .end local v2    # "i":I
    :cond_20
    return-void
.end method
