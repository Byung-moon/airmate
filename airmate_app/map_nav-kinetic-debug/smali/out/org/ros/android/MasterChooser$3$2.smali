.class Lorg/ros/android/MasterChooser$3$2;
.super Ljava/lang/Object;
.source "MasterChooser.java"

# interfaces
.implements Ljava/lang/Runnable;


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Lorg/ros/android/MasterChooser$3;->onPostExecute(Ljava/lang/Boolean;)V
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x0
    name = null
.end annotation


# instance fields
.field final synthetic this$1:Lorg/ros/android/MasterChooser$3;


# direct methods
.method constructor <init>(Lorg/ros/android/MasterChooser$3;)V
    .registers 2
    .param p1, "this$1"    # Lorg/ros/android/MasterChooser$3;

    .line 300
    iput-object p1, p0, Lorg/ros/android/MasterChooser$3$2;->this$1:Lorg/ros/android/MasterChooser$3;

    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public run()V
    .registers 3

    .line 303
    iget-object v0, p0, Lorg/ros/android/MasterChooser$3$2;->this$1:Lorg/ros/android/MasterChooser$3;

    iget-object v0, v0, Lorg/ros/android/MasterChooser$3;->this$0:Lorg/ros/android/MasterChooser;

    invoke-static {v0}, Lorg/ros/android/MasterChooser;->access$300(Lorg/ros/android/MasterChooser;)Landroid/widget/LinearLayout;

    move-result-object v0

    const/16 v1, 0x8

    invoke-virtual {v0, v1}, Landroid/widget/LinearLayout;->setVisibility(I)V

    .line 304
    return-void
.end method
