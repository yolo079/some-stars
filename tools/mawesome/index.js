const fs = require('fs');
const path = require('path');
const ejs = require('ejs');
const { Octokit } = require('@octokit/rest');
const argv = require('minimist')(process.argv.slice(2));

async function main() {
  const templatePath = argv.template || argv['template-path'] || './template/README.ejs';
  const githubName = argv['github-name'] || process.env.GITHUB_REPOSITORY_OWNER || process.env.GITHUB_ACTOR;
  const outPath = argv.out || argv.output || 'README.md';
  const token = process.env.PERSONAL_TOKEN || process.env.GITHUB_TOKEN;

  if (!githubName) {
    console.error('Missing --github-name or GITHUB_REPOSITORY_OWNER');
    process.exit(2);
  }

  const octokit = token ? new Octokit({ auth: token }) : new Octokit();

  // Example: fetch starred repos of the given user
  let stars = [];
  try {
    const per_page = 100;
    let page = 1;
    while (true) {
      const res = await octokit.activity.listReposStarredByUser({
        username: githubName,
        per_page,
        page
      });
      if (!res || !res.data) break;
      stars = stars.concat(res.data);
      if (res.data.length < per_page) break;
      page++;
    }
  } catch (err) {
    console.error('Error fetching starred repos:', (err && err.message) || err);
    process.exit(3);
  }

  // Load template
  const tplFullPath = path.resolve(templatePath);
  if (!fs.existsSync(tplFullPath)) {
    console.error('Template not found at', tplFullPath);
    process.exit(4);
  }
  const tpl = fs.readFileSync(tplFullPath, 'utf8');

  // Prepare data for template
  const data = {
    githubName,
    stars: stars.map(r => ({
      name: r.full_name,
      url: r.html_url,
      description: r.description,
      language: r.language,
      stargazers_count: r.stargazers_count,
      owner: r.owner && r.owner.login
    }))
  };

  // Render
  let out;
  try {
    out = ejs.render(tpl, data, { async: false });
  } catch (err) {
    console.error('Error rendering template:', err);
    process.exit(5);
  }

  fs.writeFileSync(outPath, out, 'utf8');
  console.log('Wrote', outPath);
}

main().catch(err => {
  console.error(err);
  process.exit(1);
});
