/* Same equations and SI units as python/physics.py. No DOM dependencies. */
const Physics = (() => {
 const specs=DATA.parameters, defaults=Object.fromEntries(specs.map(p=>[p.key,p.default]));defaults.mode='corner';
 const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
 function normalize(input={}) {
  const s={...defaults};for(const p of specs){const raw=input[p.key];if(raw===null||raw==='')throw Error('Некорректное значение: '+p.key);const v=Number(raw??p.default);if(!Number.isFinite(v))throw Error('Неконечное значение: '+p.key);s[p.key]=clamp(v,p.min,p.max);}
  s.mode=input.mode??'corner';if(!['corner','straight'].includes(s.mode))throw Error('Неизвестный режим');return s;
 }
 function coefficients(input) {
  const s=normalize(input),af=s.front*(s.mode==='straight'?.46:1),ar=s.rear*(s.mode==='straight'?.32:1),wind=1-.14*Math.abs(s.crosswind)/15;
  const front=(.38+.035*af)*s.front_area*wind,rear=(.50+.042*ar)*s.rear_area*wind;
  let floor=.95*Math.exp(-1*((s.ride-38)/42)**2);
  floor*=1-.28*clamp((25-s.ride)/5,0,1);floor*=(1+.09*s.rake)*Math.exp(-1*((s.diffuser-11)/12)**2);
  floor*=clamp(.9+s.stiffness/700,.94,1.06)*wind;
  const cl=front+rear+floor,cd=s.body_cd+.06*s.cooling+.006*af*s.front_area+.009*ar*s.rear_area+.035*(front*front+rear*rear)+.025*floor+.035*(s.crosswind/15)**2;
  const mu=s.grip*Math.exp(-1*((s.pressure-22)/10)**2)*(.65+.35*Math.exp(-1*((s.tyre_temp-85)/28)**2))*(1-.018*(s.camber-2)**2)*(1-.10*((s.stiffness-65)/60)**2);
  return [s,{front,rear,floor,cl,cd,mu}];
 }
 function evaluate(input) {
  const [s,c]=coefficients(input),G=9.81,A=1.5,CRR=.015,mass=s.mass,weight=s.front_weight/100,brake=s.brake_bias/100;
  const sf=mass*G*weight,sr=mass*G*(1-weight);
  function forces(v){const q=.5*s.density*v*v;return [q*A*c.cl,q*A*c.cd];}
  function capacity(v){const [d]=forces(v),fraction=(c.front+.43*c.floor)/c.cl,nf=sf+d*fraction,nr=sr+d*(1-fraction),transfer=.004*(s.arb-50);return [c.mu*(1-transfer)*nf*(sf/nf)**.1,c.mu*(1+transfer)*nr*(sr/nr)**.1,nf,nr];}
  let lo=0,hi=250;for(let i=0;i<60;i++){const v=(lo+hi)/2,[d,r]=forces(v);if((r+CRR*(mass*G+d))*v>s.power*1000)hi=v;else lo=v;}
  const power_limit=(lo+hi)*1.8,gear_limit=355/s.gear_ratio,vmax=Math.min(power_limit,gear_limit);
  lo=0;hi=vmax/3.6;for(let i=0;i<50;i++){const v=(lo+hi)/2,[f,r]=capacity(v),available=Math.min(f/(mass*weight),r/(mass*(1-weight)));if(v*v/s.radius>available)hi=v;else lo=v;}
  const corner_speed=(lo+hi)*1.8,v=s.speed/3.6,[down,drag]=forces(v),[cf,cr,nf,nr]=capacity(v);
  const af=cf/(mass*weight),ar=cr/(mass*(1-weight)),available=Math.min(af,ar),requested=v*v/s.radius,handling=100*(ar-af)/Math.max(af,ar);
  const rolling=CRR*(mass*G+down),drive=Math.min(s.power*1000/Math.max(v,8),mass*G*.8*s.gear_ratio,cr);
  let accel=(drive-drag-rolling)/mass;if(s.speed>=gear_limit)accel=Math.min(accel,0);
  let distance=0;for(let i=0;i<80;i++){const v0=v*i/80,v1=v*(i+1)/80,vm=(v0+v1)/2,[f,r]=capacity(vm),[,resistance]=forces(vm);let decel=Math.min(f/brake,r/(1-brake))/mass;decel*=Math.min(s.brake_pressure,1);decel+=resistance/mass;distance+=(v1*v1-v0*v0)/(2*Math.max(decel,.1));}
  const q=.5*s.density*v*v,cooling=100*s.cooling*(.55+.45*Math.min(s.speed/220,1.5))*600/s.power;
  return {down,drag,cl:c.cl,cd:c.cd,mu:c.mu,front_down:q*A*c.front,rear_down:q*A*c.rear,floor_down:q*A*c.floor,balance:(c.front+.43*c.floor)/c.cl*100,vmax,power_limit,gear_limit,corner_speed,braking:distance,accel,handling,cooling,lateral_g:available/G,required_g:requested/G,corner_margin:available-requested,front_load:nf,rear_load:nr,lock_risk:Math.max(0,s.brake_pressure-1)*100,yaw_force:.5*s.density*s.crosswind*Math.abs(s.crosswind)*2,power_used:(drag+rolling)*v};
 }
 function sweep(s,key,count=25){const p=specs.find(p=>p.key===key);count=Math.max(2,Math.floor(count));return Array.from({length:count},(_,i)=>{const value=p.min+(p.max-p.min)*i/(count-1);return {value,...evaluate({...s,[key]:value})};});}
 function sensitivity(input,metric='corner_speed'){const s=normalize(input);return specs.filter(p=>p.key!=='steer').map(p=>{const step=(p.max-p.min)*.05,low=clamp(s[p.key]-step,p.min,p.max),high=clamp(s[p.key]+step,p.min,p.max);return {key:p.key,low,high,delta:evaluate({...s,[p.key]:high})[metric]-evaluate({...s,[p.key]:low})[metric]};}).sort((a,b)=>Math.abs(b.delta)-Math.abs(a.delta));}
 function streamlines(input){const s=normalize(input),curves=[],wing=s.rear*(s.mode==='straight'?.32:1);
  for(let lane=0;lane<9;lane++){const z0=(lane-4)*.29,points=[];for(let i=0;i<33;i++){const x=-3.3+i*6.6/32,h=Math.exp(-1*((x+.2)/.95)**2),center=Math.exp(-1*(z0/.7)**2);points.push([x,.25+.7*h*center+Math.max(0,x-1.4)/1.9*wing*.006,z0+s.crosswind/15*(x+3.3)*.13+(z0>=0?1:-1)*h*.10*s.cooling]);}curves.push({points,kind:'external'});}
  for(const z of [-.3,0,.3]){const points=[];for(let i=0;i<33;i++){const x=-3.3+i*6.6/32;points.push([x,.04+(s.ride-20)*.0005+Math.max(0,x-1)*Math.tan(s.diffuser*Math.PI/180)*.45,z]);}curves.push({points,kind:'floor'});}return curves;
 }
 return {defaults,normalize,evaluate,coefficients,sweep,sensitivity,streamlines,clamp};
})();
