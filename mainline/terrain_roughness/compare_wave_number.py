"""Two single-wave clips and two wave-number plots from completed evidence only."""
from pathlib import Path
import hashlib
import json
import os

for k in ['OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS']:
    os.environ[k] = '1'
import numpy as np
import imageio.v2 as imageio
from PIL import Image, ImageDraw, ImageFont
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path('X:/Projects/Centipede_IsaacLab_RL/mainline/terrain_roughness')
BASE = ROOT/'shortwave'
DEMOS = BASE/'demos'
OUT = BASE/'displacement_roughness'
REPORT = OUT/'WAVE_NUMBER_PLOTS.json'
WAVES = [1.5, 2, 2.5, 3, 3.5, 4]
LEVELS = ['original', 'restore4', 'restore2']

def read(p): return json.loads(p.read_text(encoding='utf-8'))
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()


def extract(level, stem, cutoff, grid, metrics, vr):
    src=DEMOS/f'libo_{level}_six_waves.mp4'
    dst=DEMOS/f'{stem}.mp4'
    assert not dst.exists(), f'Output already exists: {dst}'
    entry=next(v for v in vr['videos'] if v['site']=='libo' and v['level']==level)
    assert sha(src)==entry['sha256'] and entry['waves'][0]==1.5
    run=BASE/f'simulation_v2/runs/v2/libo_{level}_low'
    trajectory_hash=sha(run/'trajectory.npz')
    assert trajectory_hash==next(v['trajectory_sha256'] for v in vr['sources'] if v['run']==str(run))
    cell=read(run/'amplitude_modulation.json')['cells'][0]
    assert cell['wave']==1.5 and cell['surface']==f'libo_{level}'
    metric=next(c for c in metrics['cells'] if (c['site'],c['level'],c['wave'])==('libo',level,1.5))
    with np.load(OUT/'forward_timeseries.npz') as z:
        times=z['t_s'];fwd=z[f'libo_{level}_w1.5']
    assert abs(fwd[600]-metric['forward_0_6_mm'])<1e-10
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',22)
    title=ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf',28)
    small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',19)
    writer=imageio.get_writer(dst,fps=20,codec='libx264',quality=8,macro_block_size=1,
                             ffmpeg_params=['-threads','2','-movflags','+faststart','-preset','fast'])
    frames=0
    try:
        with imageio.get_reader(src) as reader:
            for i,frame in enumerate(reader):
                assert frame.shape==(1760,1440,3)
                tile=Image.fromarray(frame[167:647,:720])
                canvas=Image.new('RGB',(720,720),'#f5f7fa');draw=ImageDraw.Draw(canvas)
                suffix=' | original terrain' if level=='original' else ''
                draw.text((18,9),'Libo | wave 1.5'+suffix,font=title,fill='#142f45')
                draw.text((18,45),f'Cutoff wavelength: {cutoff:g} mm | grid: {grid:g} mm',font=font,fill='#355169')
                draw.text((18,80),f't = {times[i*5]:.2f} s   |   Forward displacement = {fwd[i*5]:.2f} mm',font=font,fill='#142f45')
                canvas.paste(tile,(0,116))
                draw.text((18,609),f'A = {cell["A_deg"]:.3f} deg | B = {cell["B_deg"]:.3f} deg | fixed Sep 17 wave-4 PD',font=small,fill='#355169')
                draw.text((18,640),'Saved Isaac PhysX motion | USD OpenGL replay | 1x / 20 fps',font=small,fill='#526a7b')
                draw.text((18,671),'Camera follows body; counter uses fixed-world COM displacement.',font=small,fill='#526a7b')
                writer.append_data(np.asarray(canvas));frames+=1
    finally:
        writer.close()
    assert frames==120
    selected=[]
    with imageio.get_reader(dst) as reader:
        for count,frame in enumerate(reader,1):
            assert frame.shape==(720,720,3) and frame.std()>5
            if count in [1,61,120]:selected.append(Image.fromarray(frame))
    assert count==120
    review=Image.new('RGB',(1440,480),'white')
    for i,im in enumerate(selected):review.paste(im.resize((480,480)),(480*i,0))
    review_path=DEMOS/f'{stem}_review.jpg';review.save(review_path,quality=93)
    result=dict(video=str(dst),sha256=sha(dst),source_video=str(src),source_video_sha256=entry['sha256'],
                source_trajectory=str(run/'trajectory.npz'),source_trajectory_sha256=trajectory_hash,
                level=level,wave=1.5,environment_index=0,tile_box_xyxy=[0,167,720,647],
                cutoff_wavelength_mm=cutoff,terrain_grid_mm=grid,fps=20,frame_count=120,duration_s=6.,source_times_s=[0,5.95],
                exact_t6_forward_mm=metric['forward_0_6_mm'],full_decode_passed=True,new_physics_steps=0,
                geometry_camera_unchanged=True,source_hash_unchanged=sha(src)==entry['sha256'],review_image=str(review_path))
    print(f'Video complete: {dst.name}; 6 s displacement {metric["forward_0_6_mm"]:.3f} mm',flush=True)
    return result


def plot(metrics):
    lookup={(c['site'],c['level'],c['wave']):c for c in metrics['cells']}
    colors=['#536b80','#d48924','#247c98']
    labels=['Original: cutoff 8 mm, grid 2 mm','Restored cutoff 4 mm, grid 0.5 mm','Restored cutoff 2 mm, grid 0.5 mm']
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12,'axes.spines.top':False,'axes.spines.right':False,'axes.linewidth':.8})
    results=[]
    for site in ['libo','narvik']:
        path=OUT/f'{site}_displacement_vs_wave_number.png'
        assert not path.exists(), f'Output already exists: {path}'
        fig,ax=plt.subplots(figsize=(9,6.3))
        fig.subplots_adjust(left=.12,right=.96,bottom=.20,top=.70)
        fig.suptitle(f'{site.capitalize()}: forward displacement vs. wave number',fontsize=17,weight='bold',y=.975)
        fig.text(.5,.912,'Three shortwave-band conditions | same saved 0-6 s simulations',ha='center',fontsize=11,color='#52616f')
        data=[]
        for level,color,label,marker in zip(LEVELS,colors,labels,['o','s','^']):
            yy=[lookup[(site,level,w)]['forward_0_6_mm'] for w in WAVES]
            ax.plot(WAVES,yy,color=color,label=label,marker=marker,markersize=7,lw=2.2,markeredgecolor='white',markeredgewidth=.8)
            data.append(dict(level=level,wave_numbers=WAVES,forward_displacement_mm=yy))
        fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.53,.88),frameon=False,fontsize=10.5,ncol=1)
        ax.set(xlabel='Wave number',ylabel='Forward displacement, 0-6 s (mm)',xlim=(1.42,4.08),ylim=(0,240))
        ax.set_xticks(WAVES,[f'{w:g}' for w in WAVES]);ax.set_yticks(np.arange(0,241,40));ax.grid(alpha=.18)
        assert all(0<=y<=240 for row in data for y in row['forward_displacement_mm'])
        fig.text(.12,.075,'Whole-body COM projected on the initial forward axis (-X).',fontsize=10,color='#52616f')
        fig.text(.12,.035,'Flat-cluster amplitudes and Sep 17 wave-4 PD fixed. One saved run per condition.',fontsize=10,color='#52616f')
        fig.savefig(path,dpi=170,facecolor='white');plt.close(fig)
        with Image.open(path) as image:
            image.load();assert image.size==(1530,1071)
        results.append(dict(site=site,path=str(path),sha256=sha(path),data=data,shared_y_limits_mm=[0,240],full_decode_passed=True))
        print(f'Plot complete: {path.name}',flush=True)
    return results


if __name__=='__main__':
    assert not REPORT.exists(), 'Existing receipt must not be overwritten.'
    metrics=read(OUT/'RESULTS.json');assert metrics['complete']
    metric_hash=sha(OUT/'RESULTS.json')
    vr=read(DEMOS/'VIDEO_REPORT.json');assert vr['complete']
    videos=[extract('original','libo_original_wave1p5',8,2,metrics,vr),extract('restore4','libo_lambda4mm_wave1p5',4,.5,metrics,vr)]
    plots=plot(metrics)
    assert sha(OUT/'RESULTS.json')==metric_hash
    report=dict(complete=True,source_metrics=str(OUT/'RESULTS.json'),source_metrics_sha256=metric_hash,
                source_video_report_sha256=sha(DEMOS/'VIDEO_REPORT.json'),source_code=str(Path(__file__).resolve()),source_code_sha256=sha(Path(__file__)),
                videos=videos,plots=plots,new_physics_steps=0,amplitudes_PD_trajectories_unchanged=True,
                scope='Completed shortwave-band comparison only; this is not the proposed fixed-cutoff height sweep.')
    REPORT.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
