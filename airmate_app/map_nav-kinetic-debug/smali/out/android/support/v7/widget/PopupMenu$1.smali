.class Landroid/support/v7/widget/PopupMenu$1;
.super Ljava/lang/Object;
.source "PopupMenu.java"

# interfaces
.implements Landroid/support/v7/view/menu/MenuBuilder$Callback;


# annotations
.annotation system Ldalvik/annotation/EnclosingMethod;
    value = Landroid/support/v7/widget/PopupMenu;-><init>(Landroid/content/Context;Landroid/view/View;III)V
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x0
    name = null
.end annotation


# instance fields
.field final synthetic this$0:Landroid/support/v7/widget/PopupMenu;


# direct methods
.method constructor <init>(Landroid/support/v7/widget/PopupMenu;)V
    .registers 2
    .param p1, "this$0"    # Landroid/support/v7/widget/PopupMenu;

    .line 108
    iput-object p1, p0, Landroid/support/v7/widget/PopupMenu$1;->this$0:Landroid/support/v7/widget/PopupMenu;

    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public onMenuItemSelected(Landroid/support/v7/view/menu/MenuBuilder;Landroid/view/MenuItem;)Z
    .registers 4
    .param p1, "menu"    # Landroid/support/v7/view/menu/MenuBuilder;
    .param p2, "item"    # Landroid/view/MenuItem;

    .line 111
    iget-object v0, p0, Landroid/support/v7/widget/PopupMenu$1;->this$0:Landroid/support/v7/widget/PopupMenu;

    iget-object v0, v0, Landroid/support/v7/widget/PopupMenu;->mMenuItemClickListener:Landroid/support/v7/widget/PopupMenu$OnMenuItemClickListener;

    if-eqz v0, :cond_f

    .line 112
    iget-object v0, p0, Landroid/support/v7/widget/PopupMenu$1;->this$0:Landroid/support/v7/widget/PopupMenu;

    iget-object v0, v0, Landroid/support/v7/widget/PopupMenu;->mMenuItemClickListener:Landroid/support/v7/widget/PopupMenu$OnMenuItemClickListener;

    invoke-interface {v0, p2}, Landroid/support/v7/widget/PopupMenu$OnMenuItemClickListener;->onMenuItemClick(Landroid/view/MenuItem;)Z

    move-result v0

    return v0

    .line 114
    :cond_f
    const/4 v0, 0x0

    return v0
.end method

.method public onMenuModeChange(Landroid/support/v7/view/menu/MenuBuilder;)V
    .registers 2
    .param p1, "menu"    # Landroid/support/v7/view/menu/MenuBuilder;

    .line 119
    return-void
.end method
